
#[cfg(feature = "display")]
pub mod display;

use std::collections::VecDeque;
use std::mem::ManuallyDrop;
use std::iter::Extend;
use std::borrow::Borrow;
use std::num::NonZeroUsize;

use nalgebra as na;

// TODO: Should have some kind of `entry()` just like `BTreeMap`.

pub trait HasCoords<const N: usize> {
    fn coords(&self) -> [f32; N];

    fn point(&self) -> na::Point<f32, N> {
        self.coords().into()
    }
}

impl<const N: usize> HasCoords<N> for na::Point<f32, N> {
    #[inline]
    fn coords(&self) -> [f32; N] {
        let mut array = [0.0; N];
        array.copy_from_slice(self.coords.data.as_slice());
        array
    }
}

/// 
/// A simple implementation of ca K-D Tree, this implementation could generate very unbalanced trees which
/// would result in very inefficient queries. Use with caution in performance sensitive applications.
///
#[derive(Debug, Clone)]
pub struct KDTree<P, const N: usize> {
    nodes: Vec<Node>,
    data: Vec<P>,
}

impl<P, const N: usize> KDTree<P, N> {
    pub const ROOT_IDX: usize = 0;

    pub fn get_point(&self, idx: usize) -> &P {
        &self.data[idx]
    }

    pub fn get_point_mut(&mut self, idx: usize) -> &mut P {
        &mut self.data[idx]
    }
}

impl<P, const N: usize> KDTree<P, N> {
    #[inline]
    fn get_root(&self) -> Option<Node> {
        self.get_node(Self::ROOT_IDX)
    }

    #[inline]
    fn get_node(&self, idx: usize) -> Option<Node> {
        self.nodes.get(idx).copied()
    }

    #[inline]
    fn get_node_mut(&mut self, idx: usize) -> Option<&mut Node> {
        self.nodes.get_mut(idx)
    }

    fn add_node(&mut self, p: P) -> usize {
        let node_idx = self.nodes.len();
        let data_idx = self.data.len();
        self.data.push(p);
        self.nodes.push(Node::new(data_idx, node_idx));

        node_idx
    }
}

impl<P: HasCoords<N>, const N: usize> KDTree<P, N> {
    #[inline]
    fn get_data(&self, idx: usize) -> &P {
        &self.data[idx]
    }
}

impl<P: HasCoords<N>, const N: usize> KDTree<P, N> {
    #[inline]
    pub fn new() -> KDTree<P, N> {
        KDTree {
            nodes: Vec::new(),
            data: Vec::new(),
        }
    }

    pub fn size(&self) -> usize {
        self.nodes.len()
    }

    pub fn insert(&mut self, p: P) -> usize {
        if let Some(mut root) = self.get_root() {
            root.insert(self, p, 0).into()
        } else {
            self.add_node(p)
        }
    }

    pub fn find_nearest<Q>(&self, p: &Q) -> Option<&P>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let root = self.get_root()?;
        Some(root.find_nearest(self, p, 0))
    }

    // TODO: Should return something like a struct NodeRef(&P, usize) instead
    pub fn find_index_nearest<Q>(&self, p: &Q) -> Option<usize>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let root = self.get_root()?;
        Some(root.find_index_nearest(self, p, 0))
    }

    pub fn find_within_radius<Q>(&self, p: &Q, radius: f32) -> Vec<&P>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.get_root()
            .map(|root| root.find_within_radius(self, p, radius, 0))
            .unwrap_or_default()
    }

    pub fn find_indices_within_radius<Q>(&self, p: &Q, radius: f32) -> Vec<usize>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.get_root()
            .map(|root| root.find_indices_within_radius(self, p, radius, 0))
            .unwrap_or_default()
    }

    pub fn extend_vec(&mut self, points: Vec<P>) {
        // SAFETY: Since `ManuallyDrop<T>` is #[repr(transparent)] it is safe to transmute.
        let mut points: Vec<ManuallyDrop<P>> = unsafe { std::mem::transmute(points) };
        let slice = points.as_mut_slice();

        if let Some(mut root) = self.get_root() {
            root.extend_slice(self, slice, 0);
        } else {
            Node::from_slice(self, slice, 0);
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &P> {
        self.data.iter()
    }

    pub fn dfs(&self) -> DFSIter<P, N> {
        DFSIter::new(self)
    }
}

#[derive(Debug, Clone, Copy)]
struct Node {
    data_idx: usize,
    node_idx: usize,
    left: Option<NonZeroUsize>,
    right: Option<NonZeroUsize>,
}

impl Node {
    fn new(data_idx: usize, node_idx: usize) -> Self {
        Node { data_idx, node_idx, left: None, right: None }
    }

    #[inline(always)]
    fn get_data<'a, P, const N: usize>(&self, tree: &'a KDTree<P, N>) -> &'a P
    where
        P: HasCoords<N>,
    {
        tree.get_data(self.data_idx)
    }

    #[inline(always)]
    fn get_left<P, const N: usize>(&self, tree: &KDTree<P, N>) -> Option<Node> {
        tree.get_node(self.left?.into())
    }

    #[inline(always)]
    fn get_right<P, const N: usize>(&self, tree: &KDTree<P, N>) -> Option<Node> {
        tree.get_node(self.right?.into())
    }

    #[inline(always)]
    fn write<P, const N: usize>(&self, tree: &mut KDTree<P, N>) {
        tree.nodes[self.node_idx] = *self;
    }

    fn insert<P, const N: usize>(&mut self, tree: &mut KDTree<P, N>, p: P, depth: usize) -> NonZeroUsize
    where
        P: HasCoords<N>,
    {
        let axis = depth % N;
        let median = self.get_data(tree);

        let ret = if p.coords()[axis] <= median.coords()[axis] {
            if let Some(mut child) = self.get_left(tree) {
                 child.insert(tree, p, depth + 1)
            } else {
                let node_idx = NonZeroUsize::new(tree.add_node(p))
                    .expect("Child node cannot have index 0");

                self.left.replace(node_idx);
                node_idx
            }
        } else {
            if let Some(child) = &mut self.get_right(tree) {
                child.insert(tree, p, depth + 1)
            } else {
                let node_idx = NonZeroUsize::new(tree.add_node(p))
                    .expect("Child node cannot have index 0");

                self.right.replace(node_idx);
                node_idx
            }
        };
        self.write(tree);
        ret
    }

    fn find_nearest<'a, Q, P, const N: usize>(&self, tree: &'a KDTree<P, N>, p: &Q, depth: usize) -> &'a P
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + HasCoords<N>,
    {
        tree.get_point(self.find_index_nearest(tree, p, depth))
    }


    fn find_index_nearest<'a, Q, P, const N: usize>(&self, tree: &'a KDTree<P, N>, p: &Q, depth: usize) -> usize
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + HasCoords<N>,
    {
        let p = p.borrow();
        let axis = depth % N;

        let p_ax = p.coords()[axis];
        let median = self.get_data(tree);
        let m_ax = median.coords()[axis];

        let is_left = p_ax <= m_ax;

        // We first follow the axis comparison to get a first candidate of nearest point.
        let (fst, snd) = if is_left {
            (self.get_left(tree), self.get_right(tree))
        } else {
            (self.get_right(tree), self.get_left(tree))
        };

        let this_side_idx = fst
            .map(|child| child.find_index_nearest(tree, p, depth + 1))
            .unwrap_or(self.node_idx);

        let this_side = tree.get_point(this_side_idx);
        // If we the point that we found has a distance that is too large, then there may be 
        // points on the other side that should be considered.
        let dist_sq = na::distance_squared(&this_side.point(), &p.point());

        Some((median, self.node_idx))
            .into_iter()
            .chain(Some((this_side, this_side_idx)))
            .chain(if dist_sq > (p_ax - m_ax).powi(2) {
                snd
                    .map(|child| child.find_index_nearest(tree, p, depth + 1))
                    .map(|i| (tree.get_point(i), i))
            } else {
                None
            })
            .min_by(|&(a, _), &(b, _)| {
                let p = p.borrow().point();
                let da = na::distance_squared(&a.point(), &p);
                let db = na::distance_squared(&b.point(), &p);
                da.partial_cmp(&db).unwrap()
            })
            .unwrap()
            .1
    }

    fn find_within_radius<'a, Q, P, const N: usize>(
        &self,
        tree: &'a KDTree<P, N>,
        p: &Q,
        radius: f32,
        depth: usize
    ) -> Vec<&'a P>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + HasCoords<N>,
    {
        // This is preatty bad... It is going to allocate an entirely new vector just for this.
        self.find_indices_within_radius(tree, p, radius, depth)
            .into_iter()
            .map(|p| tree.get_point(p))
            .collect()
    }

    // TODO: It might be nicer to return an iterator. That would be more memory efficient and could also be faster.
    fn find_indices_within_radius<'a, Q, P, const N: usize>(
        &self,
        tree: &'a KDTree<P, N>,
        p: &Q,
        radius: f32,
        depth: usize
    ) -> Vec<usize>
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + HasCoords<N>,
    {
        let p = p.borrow();
        let axis = depth % N;

        let p_ax = p.coords()[axis];
        let median = self.get_data(tree);
        let m_ax = median.coords()[axis];

        let is_left = p_ax <= m_ax;

        // We first follow the axis comparison to get a first candidate of nearest point.
        let (fst, snd) = if is_left {
            (self.get_left(tree), self.get_right(tree))
        } else {
            (self.get_right(tree), self.get_left(tree))
        };

        let dist_sq = na::distance_squared(&median.point(), &p.point());

        let mut within_radius = fst
            .map(|child| child.find_indices_within_radius(tree, p, radius, depth + 1))
            .unwrap_or_default();
        
        if dist_sq <= radius {
            within_radius.push(self.node_idx);
        }

        if radius > (p_ax - m_ax).powi(2) {
            within_radius.extend(
                snd
                    .map(|child| child.find_indices_within_radius(tree, p, radius, depth + 1))
                    .unwrap_or_default()
            )
        }

        within_radius
    }

    // The caller of this function CANNOT drop any of the values in the `points` slice, since they will all
    // be consumed by the function.
    fn extend_slice<P, const N: usize>(&mut self, tree: &mut KDTree<P, N>, points: &mut [ManuallyDrop<P>], depth: usize)
    where
        P: HasCoords<N>,
    {
        if points.len() == 0 { return };
        let axis = depth % N;
        let median = self.get_data(tree);
        let m_ax = median.coords()[axis];

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| {
            a.coords()[axis]
                .partial_cmp(&b.coords()[axis])
                .unwrap()
        });

        let mid = points.partition_point(|p| p.coords()[axis] <= m_ax);
        let (left, right) = points.split_at_mut(mid);

        if let Some(mut child) = self.get_left(tree) {
            child.extend_slice(tree, left, depth + 1);
        } else {
            self.left = Node::from_slice(tree, left, depth + 1)
                .map(|node|
                    NonZeroUsize::new(node)
                        .expect("No child node can have index 0.")
                );
        }

        if let Some(mut child) = self.get_right(tree) {
            child.extend_slice(tree, right, depth + 1);
        } else {
            self.right = Node::from_slice(tree, right, depth + 1)
                .map(|node|
                    NonZeroUsize::new(node)
                        .expect("No child node can have index 0.")
                );
        }

        // We write the changes back to the tree.
        self.write(tree);
    }

    // The caller of this function CANNOT drop any of the values in the `points` slice, since they will all
    // be consumed by the function.
    fn from_slice<P, const N: usize>(tree: &mut KDTree<P, N>, points: &mut [ManuallyDrop<P>], depth: usize) -> Option<usize>
    where
        P: HasCoords<N>,
    {
        if points.len() == 0 { return None };

        let axis = depth % N;

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| {
            a.coords()[axis]
                .partial_cmp(&b.coords()[axis])
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

        let node_idx = tree.add_node(median);

        let left = Node::from_slice(tree, left, depth + 1)
            .map(|node|
                NonZeroUsize::new(node)
                    .expect("No child node can have index 0.")
            );

        let right = Node::from_slice(tree, right, depth + 1)
            .map(|node|
                NonZeroUsize::new(node)
                    .expect("No child node can have index 0.")
            );

        let node = tree.get_node_mut(node_idx).unwrap();
        node.left = left;
        node.right = right;

        Some(node_idx)
    }
}

impl<P: HasCoords<N>, const N: usize> Extend<P> for KDTree<P, N> {
    fn extend<T>(&mut self, iter: T)
    where
        T: IntoIterator<Item = P>,
    {
        for item in iter {
            self.insert(item);
        }
    }
}

pub struct DFSIter<'a, P, const N: usize> {
    tree: &'a KDTree<P, N>,
    queue: VecDeque<Node>,
}

impl<'a, P: HasCoords<N>, const N: usize> DFSIter<'a, P, N> {
    fn new(tree: &'a KDTree<P, N>) -> DFSIter<'a, P, N> {
        DFSIter { tree, queue: tree.get_root().into_iter().collect() }
    }
}

impl<'a, P: HasCoords<N>, const N: usize> Iterator for DFSIter<'a, P, N> {
    type Item = &'a P;

    fn next(&mut self) -> Option<&'a P> {
        let next = self.queue.pop_back()?;

        if let Some(left) = next.get_left(self.tree) {
            self.queue.push_back(left);
        }

        if let Some(right) = next.get_right(self.tree) {
            self.queue.push_back(right);
        }

        Some(next.get_data(self.tree))
    }
}