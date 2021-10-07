use std::borrow::Borrow;
use std::cell::{Ref, RefCell, RefMut};
use std::collections::VecDeque;
use std::iter::IntoIterator;
use std::ops::Range;

use nalgebra as na;

use crate::builder::RRTBuilder;
use crate::obstacle::Obstacle;
use crate::solvers::RRTSolver;
use crate::utils::*;
use crate::{impl_node, Path, RRTGraph, RRTResult};
use kd_tree::{
    visitor::{NearestIndexedVisitor, PreconditionVisitor, WithinRadiusIndicesVisitor},
    HasCoords, KDTree,
};

pub struct RRTStarConnectSolver;

impl<const N: usize> RRTSolver<N> for RRTStarConnectSolver {
    type Node = Node<N>;

    fn rrt_solve<O: Obstacle<N>>(builder: RRTBuilder<Self, O, N>) -> RRTResult<Self, N> {
        rrt_solve(&Cfg {
            from: builder.get_from(),
            to: builder.get_to(),
            obstacles: builder.get_obstacles(),
            random_range: builder.get_random_range(),
            step_size: builder.get_step_size(),
            update_radius: builder.get_update_radius(),
            max_iters: builder.get_max_iters(),
        })
    }
}

#[derive(Clone, Debug)]
pub struct Node<const N: usize> {
    p: na::Point<f32, N>,
    grows_from_target: bool,
    inner: RefCell<NodeInner>,
}

impl_node! {
    impl HasCoords for Node;
    impl Deref<Target = RefCell<NodeInner>> for Node(inner);
}

#[derive(Clone, Debug)]
pub struct NodeInner {
    cost: f32,
    children: Vec<usize>,
    pub connected: Option<usize>,
}

impl<const N: usize> Node<N> {
    pub fn cost(&self) -> f32 {
        self.inner().borrow().cost
    }

    pub fn connected(&self) -> Option<usize> {
        self.inner().borrow().connected
    }

    pub fn parent(&self) -> Option<usize> {
        self.inner().borrow().connected
    }

    pub fn children(&self) -> Ref<'_, [usize]> {
        Ref::map(self.inner(), |inner| inner.children.as_slice())
    }

    pub fn grows_from_target(&self) -> bool {
        self.grows_from_target
    }
}

impl<const N: usize> Node<N> {
    fn new_root(p: na::Point<f32, N>, grows_from_target: bool) -> Self {
        Node {
            p,
            grows_from_target,
            inner: RefCell::new(NodeInner {
                cost: 0.0,
                children: Vec::new(),
                connected: None,
            }),
        }
    }

    fn new<I>(
        p: na::Point<f32, N>,
        grows_from_target: bool,
        cost: f32,
        connected: usize,
        children: I,
    ) -> Self
    where
        I: IntoIterator<Item = usize>,
    {
        let node = Node {
            p,
            grows_from_target,
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

    fn inner(&self) -> Ref<NodeInner> {
        self.inner.borrow()
    }

    fn inner_mut(&self) -> RefMut<NodeInner> {
        self.inner.borrow_mut()
    }

    fn children_ref(&self) -> Ref<Vec<usize>> {
        Ref::map(self.inner(), |inner| &inner.children)
    }

    fn add_child(&self, child: usize) {
        self.inner_mut().children.push(child);
    }

    fn remove_child(&self, child: usize) {
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

struct Cfg<'a, O, const N: usize> {
    from: na::Point<f32, N>,
    to: na::Point<f32, N>,
    obstacles: &'a [O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    update_radius: f32,
    max_iters: usize,
}

fn rrt_solve<O: Obstacle<N>, const N: usize>(
    cfg: &Cfg<'_, O, N>,
) -> RRTResult<RRTStarConnectSolver, N> {
    assert!(
        cfg.update_radius >= cfg.step_size,
        "update_radius must be bigger than step_size"
    );

    let from: Node<N> = Node::new_root(cfg.from, false);
    let to: Node<N> = Node::new_root(cfg.to, true);
    let mut tree: KDTree<Node<N>, N> = KDTree::new();
    tree.insert(from.clone());
    tree.insert(to);

    // This `Option` contais a tuple of three values. The index of the tree that grow from the origin, the index
    // of the tree that grows from the target and the cost of going this way.
    let mut reach_bridge = None;
    let mut grow_from_target = false;

    for _ in 0..cfg.max_iters {
        let rnd_point = gen_random_in_range(cfg.random_range.clone());

        if let Some((_, new_node_idx)) =
            extend_and_rewire(rnd_point, &mut tree, cfg, grow_from_target)
        {
            if let Some((from_origin, from_target)) = connect(new_node_idx, &mut tree, cfg) {
                let new_cost = tree.get_point(from_origin).cost() + tree.get_point(from_target).cost();
                if let Some((_, _, cost)) = reach_bridge {
                    if new_cost < cost {
                        reach_bridge.replace((from_origin, from_target, new_cost));
                    }
                } else {
                    reach_bridge.replace((from_origin, from_target, new_cost));
                }
            }
            grow_from_target = !grow_from_target;
        }
    }

    let result = if let Some((from_origin, from_target, cost)) = reach_bridge {
        let mut waypoints = Vec::new();
        waypoints.extend(GraphPathIter::new(from_target, &tree).map(|i| {
            let node = tree.get_point(i);
            node.point()
        }));
        waypoints.reverse();
        waypoints.extend(GraphPathIter::new(from_origin, &tree).map(|i| {
            let node = tree.get_point(i);
            node.point()
        }));
        waypoints.reverse();
        Some(Path::new(waypoints, Some(cost)))
    } else {
        None
    };

    let graph = RRTGraph::new(tree.iter().cloned().collect(), from);
    RRTResult {
        n_points: tree.size(),
        graph,
        result,
    }
}

fn extend_and_rewire<O, const N: usize>(
    rnd_point: na::Point<f32, N>,
    tree: &mut KDTree<Node<N>, N>,
    cfg: &Cfg<'_, O, N>,
    grow_from_target: bool,
) -> Option<(Node<N>, usize)>
where
    O: Obstacle<N>,
{
    let vis = PreconditionVisitor::new(NearestIndexedVisitor::new(), |node: &Node<N>| {
        node.grows_from_target == grow_from_target
    });
    let (nearest_idx, nearest) = tree.query(&rnd_point, vis).unwrap();

    let direction: na::SVector<f32, N> = (rnd_point - nearest.point()).cap_magnitude(cfg.step_size);
    let new_point = nearest.point() + direction;

    // If the step point is inside any obstacle, we abandon this stepping direction.
    // NOTE: This may be a performance bottleneck. Could probably be inproved with something like
    // an AABB Tree.
    if cfg.obstacles.iter().any(|o| o.is_inside(&new_point)) {
        None
    } else {
        let vis = PreconditionVisitor::new(
            WithinRadiusIndicesVisitor::new(cfg.update_radius),
            |node: &Node<N>| node.grows_from_target == grow_from_target,
        );
        let within_radius = tree.query(&new_point, vis);

        // Find the minimum cost point P such that dist(P, new_point) + cost(P) is minimized.
        let min_cost_idx = within_radius
            .iter()
            .copied()
            .filter(|&i| {
                let p = tree.get_point(i);
                let d = na::distance(p.borrow(), &new_point);
                d <= cfg.step_size
            })
            .min_by_key(|&i| {
                let p = tree.get_point(i);
                let d = na::distance(p.borrow(), &new_point);
                (d + p.cost()).to_ord()
            })
            .unwrap_or(nearest_idx);

        let min_cost = tree.get_point(min_cost_idx);

        let cost = min_cost.cost() + na::distance(min_cost.borrow(), &new_point);
        let new_node = Node::new(
            new_point,
            grow_from_target,
            cost,
            min_cost_idx,
            std::iter::empty(),
        );
        let new_node_idx = tree.insert(new_node.clone());

        tree.get_point(min_cost_idx).add_child(new_node_idx);

        rewire(new_node_idx, within_radius, tree, cfg);
        Some((new_node, new_node_idx))
    }
}

/// Attempt to connect `target_index` to the tree coming from the other side.
/// `grow_from_target` is true only if `target_index` grows from target.
fn connect<O, const N: usize>(
    target_idx: usize,
    tree: &mut KDTree<Node<N>, N>,
    cfg: &Cfg<'_, O, N>,
) -> Option<(usize, usize)>
where
    O: Obstacle<N>,
{
    let target_node = tree.get_point(target_idx);
    let target_point = target_node.point();
    let grow_from_target = target_node.grows_from_target;

    // We first extend and rewire the tree of the other side in order to get a point that is coming
    // in the directio of `target_idx`.
    let (node, node_idx) = extend_and_rewire(target_point, tree, cfg, !grow_from_target)?;

    // Now we add many points with at most `cfg.step_size` of distance between each other in an
    // attempt to get to `target_point`.
    let mut prev = node_idx;
    let mut dir = target_point - node.point();

    while dir.magnitude() > cfg.step_size {
        /*
        let delta = dir.cap_magnitude(cfg.step_size);
        new_point = new_point + delta;

        // If we collide with an obstacle, we won't be able to reach `target_point`.
        if cfg.obstacles.iter().any(|o| o.is_inside(&new_point)) {
            return None;
        }

        let new_idx = tree.insert(Node::new(
            new_point,
            !grow_from_target,
            cost + delta.magnitude(),
            prev,
            std::iter::empty(),
        ));
        tree.get_point(prev).add_child(new_idx);
        prev = new_idx;
        */
        let (node, idx) = extend_and_rewire(target_point, tree, cfg, !grow_from_target)?;
        prev = idx;

        dir = target_point - node.point();
    }

    if grow_from_target {
        Some((prev, target_idx))
    } else {
        Some((target_idx, prev))
    }
}

struct GraphPathIter<'a, P, const N: usize> {
    curr: Option<usize>,
    tree: &'a KDTree<P, N>,
}

impl<'a, P, const N: usize> GraphPathIter<'a, P, N> {
    fn new(idx: usize, tree: &'a KDTree<P, N>) -> Self {
        GraphPathIter {
            curr: Some(idx),
            tree,
        }
    }
}

impl<'a, const N: usize> Iterator for GraphPathIter<'a, Node<N>, N> {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        let curr = self.curr?;
        let p = self.tree.get_point(curr);
        self.curr = p.connected();
        Some(curr)
    }
}

fn rewire<'a, O, I, const N: usize>(
    node_idx: usize,
    near: I,
    tree: &KDTree<Node<N>, N>,
    cfg: &Cfg<'a, O, N>,
) where
    O: Obstacle<N>,
    I: IntoIterator<Item = usize>,
{
    let node = tree.get_point(node_idx);
    for neighbor_idx in near.into_iter() {
        let neighbor = tree.get_point(neighbor_idx);
        let old_to_new = na::distance(neighbor.borrow(), node.borrow());
        if old_to_new <= cfg.step_size && neighbor.cost() > old_to_new + node.cost() {
            let cost_delta = neighbor.cost() - (old_to_new + node.cost());
            // This scope is for the lifetime of the `RefMut`
            {
                let mut inner = neighbor.inner_mut();
                if let Some(old_parent_idx) = inner.connected.replace(node_idx) {
                    let old_parent = tree.get_point(old_parent_idx);
                    old_parent.remove_child(neighbor_idx);
                }
                inner.cost -= cost_delta;
                node.add_child(neighbor_idx);
            }
            propagate_cost_update(neighbor_idx, cost_delta, tree);
        }
    }
}

fn propagate_cost_update<const N: usize>(
    node_idx: usize,
    cost_delta: f32,
    tree: &KDTree<Node<N>, N>,
) {
    let mut stack = VecDeque::new();
    stack.push_back(node_idx);

    while let Some(node_idx) = stack.pop_back() {
        let node = tree.get_point(node_idx);
        for &child_idx in node.children_ref().iter() {
            let child = tree.get_point(child_idx);
            child.inner_mut().cost -= cost_delta;
            stack.push_back(child_idx);
        }
    }
}