use std::mem::ManuallyDrop;
use std::iter::Extend;
use std::borrow::Borrow;

use nalgebra as na;

// TODO: Implement the KDTree with a vector of nodes instead of using recursive types with boxes.

pub trait HasCoords<const N: usize> {
    fn get_axis(&self, axis: usize) -> Option<f32>;

    fn point(&self) -> na::Point<f32, N> {
        let mut arr = [0.0; N];
        for i in 0..N {
            arr[i] = self.get_axis(i).unwrap();
        }
        na::Point::from_slice(&arr)
    }
}

impl<T: HasCoords<N>, const N: usize> HasCoords<N> for std::rc::Rc<T> {
    #[inline(always)]
    fn get_axis(&self, axis: usize) -> Option<f32> {
        self.as_ref().get_axis(axis)
    }
}

impl<const N: usize> HasCoords<N> for na::Point<f32, N> {
    fn get_axis(&self, axis: usize) -> Option<f32> {
        if axis < N {
            Some(self[axis])
        } else {
            None
        }
    }
}

/// 
/// A simple implementation of ca K-D Tree, this implementation could generate very unbalanced trees which
/// would result in very inefficient queries. Use with caution in performance sensitive applications.
///
#[derive(Debug, Clone)]
pub struct KDTree<P, const N: usize>(Option<KDTreeNode<P, N>>);

impl<P: HasCoords<N>, const N: usize> KDTree<P, N> {
    #[inline]
    pub fn new() -> KDTree<P, N> { KDTree(None) }

    pub fn insert(&mut self, p: P) {
        if let Some(ref mut root) = self.0 {
            root.insert(p, 0);
        } else {
            self.0.replace(KDTreeNode::new(p));
        }
    }

    pub fn find_nearest<Q>(&self, p: &Q) -> Option<&P>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let root = self.0.as_ref()?;
        Some(root.find_nearest(p, 0))
    }

    pub fn extend_vec(&mut self, points: Vec<P>) {
        // SAFETY: Since `ManuallyDrop<T>` is #[repr(transparent)] it is safe to transmute.
        let mut points: Vec<ManuallyDrop<P>> = unsafe { std::mem::transmute(points) };
        let slice = points.as_mut_slice();

        if let Some(ref mut root) = self.0 {
            root.extend_slice(slice, 0);
        } else {
            self.0 = KDTreeNode::from_slice(slice, 0);
        }
    }

    pub fn bfs(&self) -> DFSIter<P, N> {
        DFSIter::new(self)
    }
}

#[derive(Debug, Clone)]
pub struct KDTreeNode<P, const N: usize> {
    median: P,
    left: Option<Box<KDTreeNode<P, N>>>,
    right: Option<Box<KDTreeNode<P, N>>>,
}

impl<P: HasCoords<N>, const N: usize> KDTreeNode<P, N> {
    fn new(median: P) -> Self {
        KDTreeNode {
            median,
            left: None,
            right: None,
        }
    }

    fn insert(&mut self, p: P, depth: usize) {
        let axis = depth % N;

        if p.get_axis(axis).unwrap() <= self.median.get_axis(axis).unwrap() {
            if let Some(child) = &mut self.left {
                child.insert(p, depth + 1);
            } else {
                self.left.replace(Box::new(KDTreeNode::new(p)));
            }
        } else {
            if let Some(child) = &mut self.right {
                child.insert(p, depth + 1);
            } else {
                self.right.replace(Box::new(KDTreeNode::new(p)));
            }
        }
    }

    fn find_nearest<Q>(&self, p: &Q, depth: usize) -> &P
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let p = p.borrow();
        let axis = depth % N;

        let p_ax = p.get_axis(axis).unwrap();
        let m_ax = self.median.get_axis(axis).unwrap();

        let is_left = p_ax <= m_ax;

        // We first follow the axis comparison to get a first candidate of nearest point.
        let (fst, snd) = if is_left {
            (self.left.as_deref(), self.right.as_deref())
        } else {
            (self.right.as_deref(), self.left.as_deref())
        };

        let this_side = fst
            .map(|child| child.find_nearest(p, depth + 1))
            .unwrap_or(&self.median);

        // If we the point that we found has a distance that is too large, then there may be 
        // points on the other side that should be considered.
        let dist_sq = na::distance_squared(&this_side.point(), &p.point());

        Some(&self.median)
            .into_iter()
            .chain(Some(this_side))
            .chain(if dist_sq > (p_ax - m_ax).powi(2) {
                snd.map(|child| child.find_nearest(p, depth + 1))
            } else {
                None
            })
            .min_by(|a, b| {
                let p = p.borrow().point();
                let da = na::distance_squared(&a.point(), &p);
                let db = na::distance_squared(&b.point(), &p);
                da.partial_cmp(&db).unwrap()
            })
            .unwrap()
    }

    // The caller of this function CANNOT drop any of the values in the `points` slice, since they will all
    // be consumed by the function.
    fn extend_slice(&mut self, points: &mut [ManuallyDrop<P>], depth: usize) {
        if points.len() == 0 { return };

        let axis = depth % N;

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| {
            a.get_axis(axis)
                .unwrap()
                .partial_cmp(&b.get_axis(axis).unwrap())
                .unwrap()
        });

        let mut mid = points.len() / 2;

        // Finds the maximum value of `mid` such that `points[mid] == median`.
        while let Some(upper) = points.get(mid + 1) {
            // Going one upper would change the value at the `mid` index, so `mid` is right where we want.
            if upper.point() != points[mid].point() {
                break
            }
            mid += 1;
        }

        let (left, right) = points.split_at_mut(mid);

        if let Some(left_child) = self.left.as_mut() {
            left_child.extend_slice( left, depth + 1);
        } else {
            self.left = KDTreeNode::from_slice(left, depth + 1).map(Box::new);
        }

        if let Some(right_child) = self.right.as_mut() {
            right_child.extend_slice(right, depth + 1);
        } else {
            self.right = KDTreeNode::from_slice(right, depth + 1).map(Box::new);
        }
    }

    // The caller of this function CANNOT drop any of the values in the `points` slice, since they will all
    // be consumed by the function.
    fn from_slice(points: &mut [ManuallyDrop<P>], depth: usize) -> Option<KDTreeNode<P, N>> {
        if points.len() == 0 { return None };

        let axis = depth % N;

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| {
            a.get_axis(axis)
                .unwrap()
                .partial_cmp(&b.get_axis(axis).unwrap())
                .unwrap()
        });

        let mut mid = points.len() / 2;

        // Finds the maximum value of `mid` such that `points[mid] == median`.
        while let Some(upper) = points.get(mid + 1) {
            // Going one upper would change the value at the `mid` index, so `mid` is right where we want.
            if upper.point() != points[mid].point() {
                break
            }
            mid += 1;
        }

        let (left, right) = points.split_at_mut(mid);

        // This `unwrap` is ok because right has elements from indices [mid, len) and we know that `mid` is
        // a valid index (there has to be at least one element in `points`).
        let (median, right) = right.split_first_mut().unwrap();

        // SAFETY: We have split the slice, so it's not possible to access this element again in the slice.
        // The caller of this function must know not to drop this value.
        let median = unsafe { ManuallyDrop::take(median) };

        Some(KDTreeNode {
            median,
            left: KDTreeNode::from_slice(left, depth + 1).map(Box::new),
            right: KDTreeNode::from_slice(right, depth + 1).map(Box::new),
        })
    }
}

impl<P: HasCoords<N>, const N: usize> Extend<P> for KDTree<P, N> {
    fn extend<T>(&mut self, iter: T)
    where
        T: IntoIterator<Item = P>,
    {
        for item in iter {
            self.insert(item)
        }
    }
}

pub struct DFSIter<'a, P, const N: usize>(Option<Box<DFSIterInner<'a, P, N>>>);

pub struct DFSIterInner<'a, P, const N: usize> {
    node: Option<&'a KDTreeNode<P, N>>,
    left_iter: DFSIter<'a, P, N>,
    right_iter: DFSIter<'a, P, N>,
}

impl<'a, P: HasCoords<N>, const N: usize> DFSIter<'a, P, N> {
    fn new(tree: &'a KDTree<P, N>) -> DFSIter<'a, P, N> {
        DFSIter::from_node(tree.0.as_ref())
    }

    fn from_node(node: Option<&'a KDTreeNode<P, N>>) -> DFSIter<'a, P, N> {
        if let Some(node) = node {
            DFSIter(Some(Box::new(DFSIterInner {
                node: Some(node),
                left_iter: DFSIter::from_node(node.left.as_deref()),
                right_iter: DFSIter::from_node(node.right.as_deref()),
            })))
        } else {
            DFSIter(None)
        }
    }
}

impl<'a, P: HasCoords<N>, const N: usize> Iterator for DFSIter<'a, P, N> {
    type Item = &'a P;

    fn next(&mut self) -> Option<&'a P> {
        let inner = self.0.as_mut()?;

        if let Some(item) = inner.left_iter.next() {
            Some(item)
        } else if let Some(node) = inner.node.take() {
            Some(&node.median)
        } else if let Some(item) = inner.right_iter.next() {
            Some(item)
        } else {
            self.0 = None;
            None
        }
    }
}


// TODO: Should be only available by a feature.
mod ptree_impls {
    use super::*;

    impl<'a, P, const N: usize> ptree::TreeItem for &'a KDTree<P, N>
    where
        P: HasCoords<N> + Clone,
    {
        type Child = NodeDepthPair<'a, P, N>;

        fn write_self<W: std::io::Write>(&self, f: &mut W, _style: &ptree::Style) -> std::io::Result<()> {
            if let None = self.0 {
                write!(f, "No root")
            } else {
                write!(f, "Root")
            }
        }

        fn children<'b>(&'b self) -> std::borrow::Cow<[Self::Child]> {
            use std::borrow::Cow;

            if let Some(root) = self.0.as_ref() {
                Cow::Owned(vec![NodeDepthPair(Some((root, 0)))])
            } else {
                // No allocation is necessary for an empty Vec.
                Cow::Owned(Vec::new())
            }
        }
    }

    #[derive(Clone)]
    pub struct NodeDepthPair<'a, P, const N: usize>(Option<(&'a KDTreeNode<P, N>, usize)>);

    impl<'a, P, const N: usize> ptree::TreeItem for NodeDepthPair<'a, P, N>
    where
        P: HasCoords<N> + Clone,
    {
        type Child = NodeDepthPair<'a, P, N>;

        fn write_self<W: std::io::Write>(&self, f: &mut W, style: &ptree::Style) -> std::io::Result<()> {
            let NodeDepthPair(opt) = self;
            if let Some((node, depth)) = opt {
                let axis = depth % N;
                let to_paint = format!("{} at axis {}", node.median.point(), axis);
                write!(f, "{}", style.paint(to_paint))
            } else {
                write!(f, "Empty branch")
            }
        }

        fn children(&self) -> std::borrow::Cow<[Self::Child]> {
            use std::borrow::Cow;

            let NodeDepthPair(opt) = self;
            if let Some((node, depth)) = opt {

                let mut children = Vec::new();

                children.push(NodeDepthPair(
                    if let Some(left) = node.left.as_deref() {
                        Some((left, depth + 1))
                    } else {
                        None
                    }
                ));

                children.push(NodeDepthPair(
                    if let Some(right) = node.right.as_deref() {
                        Some((right, depth + 1))
                    } else {
                        None
                    }
                ));

                Cow::Owned(children)
            } else {
                Cow::Owned(Vec::new())
            }
        }
    }
}