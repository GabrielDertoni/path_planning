use nalgebra as na;
use vec_array::VecArray;

const DEGREE: usize = 5;

#[derive(Debug, Default, Clone)]
pub struct RTree<T, const N: usize> {
    root: Option<Node<N>>,
}

enum Node<T, const N: usize> {
    Inner(InnerNode<N>),
    Leaf(LeafNode<T>),
}

struct InnerNode<const N: usize> {
    children: Box<VecArray<(Node<N>, BBox<N>), DEGREE>>,
}

struct LeafNode<T> {
    data: VecArray<T, DEGREE>,
}

impl<T: Bounded, const N: usize> Node<T, N> {
    fn new_leaf(obj: T) -> Self {
        let mut data = VecArray::new();
        data.push(obj).unwrap();
        Node::Leaf(LeafNode { data })
    }

    fn new_inner() -> Self {
        Node::Inner(InnerNode {
            children: Box::new(VecArray::new()),
        })
    }
}

impl<T: Bounded, const N: usize> RTree<T, N> {
    pub fn new() -> Self {
        RTree::default()
    }

    pub fn insert(&mut self, obj: T) {
        let bbox = obj.bounding_box();
        self.insert_with_bounds(obj, &bbox);
    }
}

enum InsertResult<T> {
    Split(T),
    Fit,
}

impl<T: Bounded, const N: usize> RTree<T, N> {
    fn insert_with_bounds(&mut self, obj: T, bbox: &BBox<N>) {
        if let Some(root) = &mut self.root {
            if let InsertResult::Split(item) = root.insert_with_bounds(obj, bbox) {
                self.root.replace(Node::new_leaf(item));
            }
        } else {
            self.root.replace(Node::new_leaf(obj));
        }
    }
}

impl<T: Bounded, const N: usize> Node<T, N> {
    fn insert_with_bounds(&mut self, obj: T, bbox: &BBox<N>) -> InsertResult<T> {
        match self {
            Node::Inner(inner) => inner.insert_with_bounds(obj, bbox),
            Node::Leaf(leaf) => leaf.insert_with_bounds(obj, bbox),
        }
    }
}

impl<T: Bounded, const N: usize> InnerNode<T, N> {
    fn insert_with_bounds(&mut self, obj: T, bbox: &BBox<N>) -> InsertResult<T> {
        // TODO: Use some heuristic
        let idx = 0;
        let (child, child_bbox) = &mut self.children[idx];
        child_bbox.merge(bbox);

        if let InsertResult::Split(item) = child.insert_with_bounds(obj, bbox) {
            if let Some(slot) = self.children.slot_push() {
                slot.insert(item);
                return InsertResult::Fit;
            }
        }

        InsertResult::Split(item)
    }
}

impl<T: Bounded, const N: usize> LeafNode<T, N> {
    fn insert_with_bounds(&mut self, obj: T, _bbox: &BBox<N>) -> InsertResult<T> {
        if let Some(slot) = self.data.slot_push() {
            slot.insert(obj);
            InsertResult::Fit
        } else {
            // TODO: Use some heuristic
            let idx = DEGREE / 2;
            let right = self.data.split_array_at(idx);
            let old = right.get_slot(0)
                .unwrap()
                .replace(obj);
            InsertResult::Split(old)
        }
    }
}


pub trait Bounded<const N: usize> {
    fn bounding_box(&self) -> BBox<N>;
}

#[derive(Clone)]
pub struct BBox<const N: usize> {
    pub min_corner: na::Point<f32, N>,
    pub max_corner: na::Point<f32, N>,
}

impl<const N: usize> BBox<N> {
    pub fn new(min_corner: na::Point<f32, N>, max_corner: na::Point<f32, N>) -> BBox<N> {
        BBox {
            min_corner,
            max_corner,
        }
    }

    pub fn volume(&self) -> f32 {
        self.min_corner
            .iter()
            .zip(self.max_corner.iter())
            .map(|(min, max)| max - min)
            .product()
    }

    pub fn merge(&mut self, other: &Self) {
        use std::cmp::{min, max};

        self.min_corner = na::SVector::from_iterator(
            self.min_corner.iter()
                .zip(other.min_corner.iter())
                .map(|(&a, &b)| min(a, b))
        );

        self.max_corner = na::SVector::from_iterator(
            self.max_corner.iter()
                .zip(other.max_corner.iter())
                .map(|(&a, &b)| max(a, b))
        );
    }
}