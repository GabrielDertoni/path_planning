use std::borrow::Borrow;
use std::cell::{Ref, RefCell, RefMut};

use nalgebra as na;

use crate::impl_node;

#[derive(Clone, Debug)]
pub struct Node<const N: usize> {
    pub(crate) p: na::Point<f32, N>,
    pub(crate) inner: RefCell<NodeInner>,
}

impl_node! {
    impl HasCoords for Node;
    impl Deref<Target = RefCell<NodeInner>> for Node(inner);
}

#[derive(Clone, Debug)]
pub struct NodeInner {
    pub(crate) cost: f32,
    pub(crate) children: Vec<usize>,
    pub connected: Option<usize>,
}

impl<const N: usize> Node<N> {
    pub fn cost(&self) -> f32 {
        self.inner().borrow().cost
    }
    pub fn connected(&self) -> Option<usize> {
        self.inner().borrow().connected
    }
    pub fn children(&self) -> Ref<'_, [usize]> {
        Ref::map(self.inner(), |inner| inner.children.as_slice())
    }
}

impl<const N: usize> Node<N> {
    pub(crate) fn new_root(p: na::Point<f32, N>) -> Self {
        Node {
            p,
            inner: RefCell::new(NodeInner {
                children: Vec::new(),
                cost: 0.0,
                connected: None,
            }),
        }
    }

    pub(crate) fn new<I>(p: na::Point<f32, N>, cost: f32, connected: usize, children: I) -> Self
    where
        I: IntoIterator<Item = usize>,
    {
        let node = Node {
            p,
            inner: RefCell::new(NodeInner {
                cost,
                children: Vec::new(),
                connected: Some(connected),
            }),
        };

        for child in children {
            node.add_child(child);
        }

        node
    }

    pub(crate) fn inner(&self) -> Ref<NodeInner> {
        self.inner.borrow()
    }

    pub(crate) fn inner_mut(&self) -> RefMut<NodeInner> {
        self.inner.borrow_mut()
    }

    pub(crate) fn children_ref(&self) -> Ref<Vec<usize>> {
        Ref::map(self.inner(), |inner| &inner.children)
    }

    pub(crate) fn add_child(&self, child: usize) {
        self.inner_mut().children.push(child);
    }

    pub(crate) fn remove_child(&self, child: usize) {
        let children = &mut self.inner_mut().children;
        let idx = children
            .iter()
            .position(|&el| el == child)
            .expect("child not found");

        children.swap_remove(idx);
    }
}

impl<const N: usize> Borrow<na::Point<f32, N>> for Node<N> {
    #[inline]
    fn borrow(&self) -> &na::Point<f32, N> {
        &self.p
    }
}