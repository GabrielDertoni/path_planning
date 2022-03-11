#![feature(allocator_api, slice_ptr_get, ptr_as_uninit)]

#[cfg(feature = "display")]
pub mod display;

pub mod visitor;

use std::alloc::{Allocator, Global, Layout};
use std::borrow::Borrow;
use std::collections::VecDeque;
use std::iter::Extend;
use std::marker::PhantomData;
use std::mem::ManuallyDrop;
use std::ptr::NonNull;

use nalgebra as na;

pub use visitor::Visitor;

// TODO: Should have some kind of `entry()` just like `BTreeMap`.
// TODO: Rename `add_node` to `add_point`.

pub trait HasCoords<const N: usize> {
    fn coords(&self) -> [f32; N];

    fn point(&self) -> na::Point<f32, N> {
        self.coords().into()
    }

    fn get_coord(&self, axis: usize) -> f32 {
        self.coords()[axis]
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
/// A simple implementation of a K-D Tree, this implementation could generate very unbalanced trees which
/// would result in very inefficient queries. Use with caution in performance sensitive applications.
///
pub struct KDTree<P, const N: usize> {
    alloc: typed_arena::Arena<Node<P, N>>,
    root: Option<NonNull<Node<P, N>>>,
}

impl<P, const N: usize> KDTree<P, N>
where
    P: HasCoords<N> + Sized,
{
    fn get_root(&self) -> Option<&Node<P, N>> {
        // SAFETY: `root` is valid for the lifetime of the arena, which is the lifetime of self.
        unsafe { self.root.map(|root| root.as_ref()) }
    }

    fn get_root_mut(&mut self) -> Option<&mut Node<P, N>> {
        // SAFETY: `root` is valid for the lifetime of the arena, which is the lifetime of self.
        unsafe { self.root.map(|mut root| root.as_mut()) }
    }
}

impl<P, const N: usize> KDTree<P, N>
where
    P: HasCoords<N> + Sized,
{
    pub const ROOT_IDX: usize = 0;

    #[inline]
    pub fn new() -> KDTree<P, N> {
        KDTree {
            root: None,
            alloc: typed_arena::Arena::new(),
        }
    }

    pub fn insert(&mut self, p: P) {
        let node = NonNull::from(self.alloc.alloc(Node::new(p)));
        if let Some(root) = self.get_root_mut() {
            // SAFETY: All nodes are valid.
            unsafe { root.insert(node, 0); }
        } else {
            self.root.replace(node);
        }
    }

    pub fn query<'a, V, Q>(&'a self, p: &Q, mut vis: V) -> V::Result
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + Sized,
        V: Visitor<'a, P>,
    {
        if let Some(root) = self.get_root() {
            // SAFETY: All nodes are valid.
            unsafe { root.query(&mut vis, p, 0); }
        }
        vis.result()
    }

    pub fn extend_vec<'a>(&'a mut self, points: Vec<P>) {

        // SAFETY: Since `ManuallyDrop<T>` is #[repr(transparent)] it is sound to transmute.
        let mut points: Vec<ManuallyDrop<P>> = unsafe { std::mem::transmute(points) };
        let slice = points.as_mut_slice();

        let KDTree { ref mut root, ref alloc, .. } = self;

        if let Some(root) = root {
            // SAFETY: All nodes are valid.
            unsafe{ root.as_mut().extend_slice(alloc, slice, 0); }
        } else {
            Node::from_slice(alloc, slice, 0);
        }
    }

    /*
    pub fn iter(&self) -> impl Iterator<Item = &P> {
        self.data.iter()
            .filter_map(|el| match el {
                Slot::Occupied(el) => Some(el),
                Slot::Vacant {..}  => None,
            })
    }

    pub fn dfs(&self) -> DFSIter<P, A, N> {
        DFSIter::new(self)
    }
    */

    pub fn depth(&self) -> usize {
        self.get_root()
            .map(|root| {
                // SAFETY: All nodes are valid.
                unsafe {
                    root.depth(0)
                }
            })
            .unwrap_or(0)
    }

    pub fn size(&self) -> usize {
        self.alloc.len()
    }

    /// Rebuilds the KD-Tree into a balanced tree. **This operation will invalidate every node
    /// index aquired before the rebuilding**.
    pub fn rebuild<'a>(&'a mut self) {
        fn rec_rebuild<P, const N: usize>(
            // This `ManuallyDrop` is just to signal that some care must be taken when accessing
            // these values. The values should only be moved out of the `ManuallyDrop` once.
            // Alternatively an `Option` could be used, but that would require extra runtime
            // checks.
            nodes: &mut [NonNull<Node<P, N>>],
            depth: usize,
        ) -> Option<NonNull<Node<P, N>>>
        where
            P: HasCoords<N>,
        {
            // SAFETY: The lifetime of `nodes[i]` is the lifetime of `self`. This function is
            // only called within this lifetime.
            //
            if nodes.is_empty() {
                return None;
            }

            let mut mid = nodes.len() / 2;
            let axis = depth % N;

            nodes.sort_unstable_by(|a, b| {
                unsafe {
                    let a = a.as_ref().data.get_coord(axis);
                    let b = b.as_ref().data.get_coord(axis);
                    a.partial_cmp(&b).unwrap()
                }
            });

            // Finds the maximum value of `mid` such that `points[mid] == median`.
            while let Some(upper) = nodes.get(mid + 1) {
                unsafe {
                    let mid_point = nodes[mid].as_ref().data.point();
                    let upper = upper.as_ref().data.point();
                    // Going one upper would change the value at the `mid` index, so `mid` is right where we want.
                    if upper != mid_point {
                        break;
                    }
                }
                mid += 1;
            }

            let (left, right) = nodes.split_at_mut(mid);

            // This `unwrap` is ok because right has elements from indices [mid, len) and we know that `mid` is
            // a valid index (there has to be at least one element in `points`).
            let (&mut mut median, right) = right.split_first_mut().unwrap();

            unsafe {
                median.as_mut().left = rec_rebuild(left, depth + 1);
                median.as_mut().right = rec_rebuild(right, depth + 1);
            }

            Some(median)
        }

        let mut nodes: Box<[_]> = self.alloc.iter_mut().map(NonNull::from).collect();
        self.root = rec_rebuild(nodes.as_mut(), 0);
    }
}

#[derive(Debug)]
pub struct Node<P, const N: usize> {
    data: P,
    left: Option<NonNull<Node<P, N>>>,
    right: Option<NonNull<Node<P, N>>>,
}

impl<P, const N: usize> Node<P, N>
where
    P: HasCoords<N>,
{
    fn new(data: P) -> Self {
        Node {
            data,
            left: None,
            right: None,
        }
    }

    /// SAFETY: The caller must guarantee that `node` and `self` are valid as well as all of the
    /// children of `self`.
    unsafe fn insert(&mut self, node: NonNull<Node<P, N>>, depth: usize) {
        let axis = depth % N;
        let median = &self.data;

        if node.as_ref().data.get_coord(axis) <= median.get_coord(axis) {
            if let Some(mut child) = self.left {
                child.as_mut().insert(node, depth + 1);
            } else {
                self.left.replace(node);
            }
        } else if let Some(mut child) = self.right {
            child.as_mut().insert(node, depth + 1);
        } else {
            self.right.replace(node);
        }
    }

    /// SAFETY: The caller must guarantee that `self` is valid as well as its its children of
    /// `self`.
    unsafe fn query<'a, V, Q>(
        &'a self,
        visitor: &mut V,
        p: &Q,
        depth: usize,
    )
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + HasCoords<N>,
        V: Visitor<'a, P>,
    {
        let p = p.borrow();
        let axis = depth % N;

        let p_ax = p.get_coord(axis);
        let median = &self.data;
        let m_ax = median.get_coord(axis);

        let is_left = p_ax <= m_ax;

        // We first follow the axis comparison to get a first candidate of nearest point.
        let (fst, snd) = if is_left {
            (self.left, self.right)
        } else {
            (self.right, self.left)
        };

        if let Some(child) = fst {
            child.as_ref().query(visitor, p, depth + 1);
        }

        if na::distance_squared(&median.point(), &p.point()) <= visitor.radius_sq(p) {
            visitor.accept(median, p);
        }

        if visitor.radius_sq(p) > (p_ax - m_ax).abs().powi(2) {
            if let Some(child) = snd {
                child.as_ref().query(visitor, p, depth + 1);
            }
        }
    }

    // SAFETY: The caller of this function CANNOT drop any of the values in the `points` slice,
    // since they will all be consumed by the function. The caller must also guarantee that `self`
    // and all of its children are valid.
    unsafe fn extend_slice(
        &mut self,
        allocator: &typed_arena::Arena<Self>,
        points: &mut [ManuallyDrop<P>],
        depth: usize,
    ) {
        if points.is_empty() {
            return;
        };
        let axis = depth % N;
        let median = &self.data;
        let m_ax = median.get_coord(axis);

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| a.get_coord(axis).partial_cmp(&b.get_coord(axis)).unwrap());

        let mid = points.partition_point(|p| p.get_coord(axis) <= m_ax);
        let (left, right) = points.split_at_mut(mid);

        if let Some(mut child) = self.left {
            child.as_mut().extend_slice(allocator, left, depth + 1);
        } else {
            self.left = Node::from_slice(allocator, left, depth + 1);
        }

        if let Some(mut child) = self.right {
            child.as_mut().extend_slice(allocator, right, depth + 1);
        } else {
            self.right = Node::from_slice(allocator, right, depth + 1);
        }
    }

    // SAFETY: The caller of this function CANNOT drop any of the values in the `points` slice,
    // since they will all be consumed by the function.
    fn from_slice(
        allocator: &typed_arena::Arena<Self>,
        points: &mut [ManuallyDrop<P>],
        depth: usize,
    ) -> Option<NonNull<Node<P, N>>>
    {
        if points.is_empty() {
            return None;
        }

        let axis = depth % N;

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| a.get_coord(axis).partial_cmp(&b.get_coord(axis)).unwrap());

        let mut mid = points.len() / 2;

        // Finds the maximum value of `mid` such that `points[mid] == median`.
        while let Some(upper) = points.get(mid + 1) {
            // Going one upper would change the value at the `mid` index, so `mid` is right where we want.
            if upper.point() != points[mid].point() {
                break;
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

        let node = allocator.alloc(Node::new(median));

        node.left = Node::from_slice(allocator, left, depth + 1);
        node.right = Node::from_slice(allocator, right, depth + 1);

        Some(NonNull::from(node))
    }

    /// SAFETY: The caller must guarantee that `self` and all of its children are valid.
    unsafe fn depth(&self, depth: usize) -> usize {
        self.left
            .map(|left| left.as_ref().depth(depth + 1))
            .and_then(|left_depth| {
                self.right
                    .map(|right| right.as_ref().depth(depth + 1))
                    .map(|right_depth| right_depth.max(left_depth))
            })
            .unwrap_or(depth)
    }
}

/*
impl<'alloc, P, A, const N: usize> Extend<P> for KDTree<'alloc, P, N, A>
where
    P: HasCoords<N>,
    A: TypedAllocator<Node<'alloc, P, N>> + 'alloc,
{
    fn extend<T>(&mut self, iter: T)
    where
        T: IntoIterator<Item = P>,
    {
        for item in iter {
            self.insert(item);
        }
    }
}

pub struct DFSIter<'a, P, A: Allocator + Copy, const N: usize> {
    tree: &'a KDTree<P, N, A>,
    queue: VecDeque<Node<P, N>>,
}

impl<'a, P, A, const N: usize> DFSIter<'a, P, A, N>
where
    P: HasCoords<N>,
    A: Allocator + Copy,
{
    fn new(tree: &'a KDTree<P, N, A>) -> DFSIter<'a, P, A, N> {
        DFSIter {
            tree,
            queue: tree.get_root().into_iter().collect(),
        }
    }
}

impl<'a, P, A, const N: usize> Iterator for DFSIter<'a, P, A, N>
where
    P: HasCoords<N>,
    A: Allocator + Copy,
{
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
*/

#[cfg(test)]
mod test {
    use nalgebra as na;

    use super::*;
    use crate::visitor::*;

    fn point2(x: f32, y: f32) -> na::Point2<f32> {
        na::Point2::new(x, y)
    }

    #[test]
    fn test_nearest_vistor() {
        let points = vec![point2(0.0, 0.0), point2(1.0, 2.0), point2(3.0, 4.0)];
        let ref_point = point2(1.5, 1.5);
        let mut tree = KDTree::new();
        tree.extend_vec(points.clone());

        let vis = NearestVisitor::new();

        assert_eq!(
            tree.query(&ref_point, vis),
            points.iter()
                .min_by(|a, b| {
                    let da = na::distance(a, &ref_point);
                    let db = na::distance(b, &ref_point);
                    da.partial_cmp(&db).unwrap()
                })
        );
    }

    #[test]
    fn test_within_radius_visitor() {
        let points = vec![point2(0.0, 0.0), point2(1.0, 2.0), point2(3.0, 4.0)];
        let radius = 0.6;
        let ref_point = point2(1.5, 1.5);
        let mut tree = KDTree::new();
        tree.extend_vec(points.clone());

        let vis = WithinRadiusVisitor::new(radius);

        assert_eq!(
            tree.query(&ref_point, vis),
            points.iter()
                .filter(|a| {
                    let da = na::distance(a, &ref_point);
                    da < radius
                })
                .collect::<Vec<_>>()
        );
    }
}
