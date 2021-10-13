#[cfg(feature = "display")]
pub mod display;

pub mod visitor;

use std::borrow::Borrow;
use std::collections::VecDeque;
use std::iter::Extend;
use std::marker::PhantomData;
use std::mem::ManuallyDrop;
use std::num::NonZeroUsize;

use nalgebra as na;

pub use visitor::Visitor;

// TODO: Should have some kind of `entry()` just like `BTreeMap`.
// TODO: Rename `add_node` to `add_point`.

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
    nodes: Vec<Slot<Node<P, N>>>,
    data: Vec<Slot<P>>,
    next_node_vacant: usize,
    next_data_vacant: usize,
    size: usize,
}

impl<P, const N: usize> KDTree<P, N>
where
    P: HasCoords<N> + Sized,
{
    #[inline]
    fn get_root(&self) -> Option<Node<P, N>> {
        self.get_node(Self::ROOT_IDX)
    }

    #[inline]
    fn get_node(&self, idx: usize) -> Option<Node<P, N>> {
        self.nodes.get(idx).map(|&el| el.unwrap())
    }

    #[inline]
    fn get_node_mut(&mut self, idx: usize) -> Option<&mut Node<P, N>> {
        self.nodes.get_mut(idx).map(|el| el.as_mut().unwrap())
    }

    fn get_vacant_slot(&mut self) -> (VacantSlot<'_, Node<P, N>>, VacantSlot<'_, P>) {
        let KDTree { next_node_vacant, next_data_vacant, nodes, data, .. } = self;
        let len = nodes.len();

        let node_slot = if *next_node_vacant == len {
            nodes.push(Slot::Vacant { next_vacant: len + 1 });
            nodes[len].unwrap_vacant()
        } else {
            nodes[*next_node_vacant].unwrap_vacant()
        };

        let data_slot = if *next_data_vacant == len {
            data.push(Slot::Vacant { next_vacant: len + 1 });
            data[len].unwrap_vacant()
        } else {
            data[*next_data_vacant].unwrap_vacant()
        };

        (node_slot, data_slot)
    }

    fn add_node(&mut self, p: P) -> (usize, usize) {
        let node_idx = self.next_node_vacant;
        let data_idx = self.next_data_vacant;

        let (node_slot, data_slot) = self.get_vacant_slot();
        let next_node_vacant = node_slot.put(Node::new(data_idx, node_idx));
        let next_data_vacant = data_slot.put(p);

        self.next_node_vacant = next_node_vacant;
        self.next_data_vacant = next_data_vacant;
        self.size += 1;

        (node_idx, data_idx)
    }
}

impl<P: HasCoords<N>, const N: usize> KDTree<P, N> {
    #[inline]
    fn get_data(&self, idx: usize) -> &P {
        self.data[idx].as_ref().unwrap()
    }
}

impl<P, const N: usize> KDTree<P, N>
where
    P: HasCoords<N> + Sized,
{
    pub const ROOT_IDX: usize = 0;

    pub fn get_point(&self, idx: usize) -> &P {
        self.data[idx].as_ref().unwrap()
    }

    pub fn get_point_mut(&mut self, idx: usize) -> &mut P {
        self.data[idx].as_mut().unwrap()
    }

    #[inline]
    pub fn new() -> KDTree<P, N> {
        KDTree {
            nodes: Vec::new(),
            data: Vec::new(),
            next_node_vacant: 0,
            next_data_vacant: 0,
            size: 0,
        }
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn insert(&mut self, p: P) -> usize {
        if let Some(mut root) = self.get_root() {
            root.insert(self, p, 0).into()
        } else {
            self.add_node(p).1
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

    pub fn query<'a, V, Q>(&'a self, p: &Q, mut vis: V) -> V::Result
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + Sized,
        V: Visitor<'a, P>,
    {
        if let Some(root) = self.get_root() {
            root.query(&mut vis, self, p, 0);
        }
        vis.result()
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
            .filter_map(|el| match el {
                Slot::Occupied(el) => Some(el),
                Slot::Vacant {..}  => None,
            })
    }

    pub fn dfs(&self) -> DFSIter<P, N> {
        DFSIter::new(self)
    }

    pub fn rebuild(&mut self) {
        fn rec_rebuild<P: HasCoords<N>, const N: usize>(nodes: &mut [Node<P, N>], tree: &mut KDTree<P, N>, depth: usize) -> Option<usize> {
            if nodes.is_empty() {
                return None;
            }

            let mut mid = nodes.len() / 2;
            let axis = depth % N;

            nodes.sort_unstable_by(|a, b| {
                let a = tree.get_point(a.data_idx).coords()[axis];
                let b = tree.get_point(b.data_idx).coords()[axis];
                a.partial_cmp(&b).unwrap()
            });

            // Finds the maximum value of `mid` such that `points[mid] == median`.
            while let Some(upper) = nodes.get(mid + 1) {
                let mid_point = tree.get_point(nodes[mid].data_idx).point();
                let upper = tree.get_point(upper.data_idx).point();
                // Going one upper would change the value at the `mid` index, so `mid` is right where we want.
                if upper != mid_point {
                    break;
                }
                mid += 1;
            }

            let (left, right) = nodes.split_at_mut(mid);

            // This `unwrap` is ok because right has elements from indices [mid, len) and we know that `mid` is
            // a valid index (there has to be at least one element in `points`).
            let (&mut mut median, right) = right.split_first_mut().unwrap();

            let vacant_slot = tree.nodes[tree.next_node_vacant].unwrap_vacant();
            let node_idx = tree.next_node_vacant;
            median.node_idx = node_idx;
            tree.next_node_vacant = vacant_slot.put(median);

            let left = rec_rebuild(left, tree, depth + 1)
                .map(|node| NonZeroUsize::new(node).expect("No child node can have index 0."));

            let right = rec_rebuild(right, tree, depth + 1)
                .map(|node| NonZeroUsize::new(node).expect("No child node can have index 0."));

            let node = tree.get_node_mut(node_idx).unwrap();
            node.left = left;
            node.right = right;

            Some(node_idx)
        }

        // The node of index 0 will be the first in the `nodes` vec. This is necessary because when we
        // recursivelly rebuild the tree, we need to make so that the first node to be put on the nodes
        // array is the root node so we can keep using `NonZeroUsize`. The reverse is necessary because
        // then, index 0 will be the last to go on `self.next_node_vacant`, which is where the first node
        // will be reinserted.
        let mut nodes: Vec<_> = self.nodes.iter_mut()
            .enumerate()
            .rev()
            .filter_map(|(i, slot)| {
                if slot.is_occupied() {
                    let next_vacant = self.next_node_vacant;
                    let vacant_slot = Slot::Vacant { next_vacant };
                    self.next_node_vacant = i;
                    Some(std::mem::replace(slot, vacant_slot).unwrap())
                } else {
                    None
                }
            })
            .collect();

        let root_idx = rec_rebuild(nodes.as_mut(), self, 0);
        assert_eq!(root_idx, Some(0));
    }
}

impl<P, const N: usize> Default for KDTree<P, N>
where
    P: HasCoords<N>,
{
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
enum Slot<T> {
    Occupied(T),
    Vacant {
        next_vacant: usize
    },
}

impl<T> Slot<T> {
    fn unwrap(self) -> T {
        match self {
            Slot::Occupied(val) => val,
            _                   => panic!("unwrapped vacant entry"),
        }
    }

    fn unwrap_vacant(&mut self) -> VacantSlot<'_, T> {
        VacantSlot::new(self)
            .expect("unwrapped vacant on occupied entry")
    }

    fn as_ref(&self) -> Slot<&T> {
        match *self {
            Slot::Occupied(ref val)      => Slot::Occupied(val),
            Slot::Vacant { next_vacant } => Slot::Vacant { next_vacant },
        }
    }

    fn as_mut(&mut self) -> Slot<&mut T> {
        match *self {
            Slot::Occupied(ref mut val)  => Slot::Occupied(val),
            Slot::Vacant { next_vacant } => Slot::Vacant { next_vacant },
        }
    }

    fn replace(&mut self, new: T) -> Slot<T> {
        std::mem::replace(self, Slot::Occupied(new))
    }

    fn is_occupied(&self) -> bool {
        match *self {
            Slot::Occupied(_) => true,
            Slot::Vacant{..}  => false,
        }
    }
}

struct VacantSlot<'a, T> {
    slot: &'a mut Slot<T>,
}

impl<'a, T> VacantSlot<'a, T> {
    fn new(slot: &'a mut Slot<T>) -> Option<Self> {
        match *slot {
            Slot::Vacant {..}  => Some(VacantSlot { slot }),
            Slot::Occupied(..) => None,
        }
    }

    fn next_vacant(&self) -> usize {
        match *self.slot {
            Slot::Vacant { next_vacant } => next_vacant,
            _ => unreachable!(),
        }
    }

    fn put(self, val: T) -> usize {
        let old = std::mem::replace(self.slot, Slot::Occupied(val));
        match old {
            Slot::Vacant { next_vacant } => next_vacant,
            _ => unreachable!(),
        }
    }
}

#[derive(Debug)]
struct Node<P, const N: usize> {
    data_idx: usize,
    node_idx: usize,
    left: Option<NonZeroUsize>,
    right: Option<NonZeroUsize>,
    _marker: PhantomData<P>,
}

// For some reason it didn't work deriving `Clone` and `Copy` directly.
impl<P, const N: usize> Clone for Node<P, N> {
    fn clone(&self) -> Self {
        Node {
            data_idx: self.data_idx,
            node_idx: self.node_idx,
            left: self.left,
            right: self.right,
            _marker: self._marker,
        }
    }
}
impl<P, const N: usize> Copy for Node<P, N> {}

impl<P, const N: usize> Node<P, N>
where
    P: HasCoords<N>,
{
    fn new(data_idx: usize, node_idx: usize) -> Self {
        Node {
            data_idx,
            node_idx,
            left: None,
            right: None,
            _marker: PhantomData,
        }
    }

    #[inline(always)]
    fn get_data<'a>(&self, tree: &'a KDTree<P, N>) -> &'a P {
        tree.get_data(self.data_idx)
    }

    #[inline(always)]
    fn get_left(&self, tree: &KDTree<P, N>) -> Option<Self> {
        tree.get_node(self.left?.into())
    }

    #[inline(always)]
    fn get_right(&self, tree: &KDTree<P, N>) -> Option<Self> {
        tree.get_node(self.right?.into())
    }

    #[inline(always)]
    fn write(&self, tree: &mut KDTree<P, N>) {
        tree.nodes[self.node_idx].replace(*self);
    }

    fn insert(&mut self, tree: &mut KDTree<P, N>, p: P, depth: usize) -> NonZeroUsize
    where
        P: HasCoords<N>,
    {
        let axis = depth % N;
        let median = self.get_data(tree);

        let ret = if p.coords()[axis] <= median.coords()[axis] {
            if let Some(mut child) = self.get_left(tree) {
                child.insert(tree, p, depth + 1)
            } else {
                let node_idx = NonZeroUsize::new(tree.add_node(p).0)
                    .expect("Child node cannot have index 0");

                self.left.replace(node_idx);
                node_idx
            }
        } else if let Some(child) = &mut self.get_right(tree) {
            child.insert(tree, p, depth + 1)
        } else {
            let node_idx = NonZeroUsize::new(tree.add_node(p).0)
                .expect("Child node cannot have index 0");

            self.right.replace(node_idx);
            node_idx
        };
        self.write(tree);
        ret
    }

    fn find_nearest<'a, Q>(&self, tree: &'a KDTree<P, N>, p: &Q, depth: usize) -> &'a P
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q> + HasCoords<N>,
    {
        tree.get_point(self.find_index_nearest(tree, p, depth))
    }

    fn find_index_nearest<'a, Q>(&self, tree: &'a KDTree<P, N>, p: &Q, depth: usize) -> usize
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
                snd.map(|child| child.find_index_nearest(tree, p, depth + 1))
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

    fn find_within_radius<'a, Q>(
        &self,
        tree: &'a KDTree<P, N>,
        p: &Q,
        radius: f32,
        depth: usize,
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
    fn find_indices_within_radius<'a, Q>(
        &self,
        tree: &'a KDTree<P, N>,
        p: &Q,
        radius: f32,
        depth: usize,
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

        if dist_sq <= radius.powi(2) {
            within_radius.push(self.node_idx);
        }

        if radius > (p_ax - m_ax).abs() {
            within_radius.extend(
                snd.map(|child| child.find_indices_within_radius(tree, p, radius, depth + 1))
                    .unwrap_or_default(),
            )
        }

        within_radius
    }

    pub fn query<'a, V, Q>(
        &self,
        visitor: &mut V,
        tree: &'a KDTree<P, N>,
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

        if let Some(child) = fst {
            child.query(visitor, tree, p, depth + 1);
        }

        let dist_sq = na::distance_squared(&median.point(), &p.point());
        if dist_sq <= visitor.radius(p).powi(2) {
            visitor.accept(median, self.data_idx, p);
        }

        if visitor.radius(p) > (p_ax - m_ax).abs() {
            if let Some(child) = snd {
                child.query(visitor, tree, p, depth + 1);
            }
        }
    }

    // The caller of this function CANNOT drop any of the values in the `points` slice, since they will all
    // be consumed by the function.
    fn extend_slice(
        &mut self,
        tree: &mut KDTree<P, N>,
        points: &mut [ManuallyDrop<P>],
        depth: usize,
    ) {
        if points.is_empty() {
            return;
        };
        let axis = depth % N;
        let median = self.get_data(tree);
        let m_ax = median.coords()[axis];

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| a.coords()[axis].partial_cmp(&b.coords()[axis]).unwrap());

        let mid = points.partition_point(|p| p.coords()[axis] <= m_ax);
        let (left, right) = points.split_at_mut(mid);

        if let Some(mut child) = self.get_left(tree) {
            child.extend_slice(tree, left, depth + 1);
        } else {
            self.left = Node::from_slice(tree, left, depth + 1)
                .map(|node| NonZeroUsize::new(node).expect("No child node can have index 0."));
        }

        if let Some(mut child) = self.get_right(tree) {
            child.extend_slice(tree, right, depth + 1);
        } else {
            self.right = Node::from_slice(tree, right, depth + 1)
                .map(|node| NonZeroUsize::new(node).expect("No child node can have index 0."));
        }

        // We write the changes back to the tree.
        self.write(tree);
    }

    // The caller of this function CANNOT drop any of the values in the `points` slice, since they will all
    // be consumed by the function.
    fn from_slice(
        tree: &mut KDTree<P, N>,
        points: &mut [ManuallyDrop<P>],
        depth: usize,
    ) -> Option<usize>
    {
        if points.is_empty() {
            return None;
        }

        let axis = depth % N;

        // Lots of unwraps... Maybe there is a better way.
        points.sort_unstable_by(|a, b| a.coords()[axis].partial_cmp(&b.coords()[axis]).unwrap());

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

        let node_idx = tree.add_node(median).0;

        let left = Node::from_slice(tree, left, depth + 1)
            .map(|node| NonZeroUsize::new(node).expect("No child node can have index 0."));

        let right = Node::from_slice(tree, right, depth + 1)
            .map(|node| NonZeroUsize::new(node).expect("No child node can have index 0."));

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
    queue: VecDeque<Node<P, N>>,
}

impl<'a, P: HasCoords<N>, const N: usize> DFSIter<'a, P, N> {
    fn new(tree: &'a KDTree<P, N>) -> DFSIter<'a, P, N> {
        DFSIter {
            tree,
            queue: tree.get_root().into_iter().collect(),
        }
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