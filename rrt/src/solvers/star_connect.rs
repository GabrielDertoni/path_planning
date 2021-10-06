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

/// An RRT* solver that is somewhat fast, but may be innacurate with respect to the cost of
/// getting to the goal. This happens because only a constant number of closest neighbors are
/// stored for each node, meaning that it is possible that a node is connected to a parent
/// but the parent doesn't have a reference to this neighbor. This means that if the parent
/// is updated, it will not update the child's cost, even though the path through the child
/// now has a different cost.
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
            target_radius: builder.get_target_radius(),
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
    other_side: Option<usize>,
    children: Vec<usize>,
    pub connected: Option<usize>,
}

impl<const N: usize> Node<N> {
    pub fn cost(&self) -> f32 {
        self.inner().borrow().cost
    }

    pub fn connected(&self) -> Option<usize> {
        if self.grows_from_target {
            self.inner().borrow().other_side
        } else {
            self.inner().borrow().connected
        }
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
                other_side: None,
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
                other_side: None,
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
    target_radius: f32,
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
    let to_idx = tree.insert(to);

    let mut reached_idx = None;
    let mut grow_from_target = false;

    for _ in 0..cfg.max_iters {
        let rnd_point = gen_random_in_range(cfg.random_range.clone());

        if let Some((_, new_node_idx)) =
            extend_and_rewire(rnd_point, &mut tree, cfg, grow_from_target)
        {
            if let Some(reached) = connect(new_node_idx, &mut tree, cfg, grow_from_target) {
                reached_idx.replace(reached);
                break;
            }
            grow_from_target = !grow_from_target;
        }
    }

    let result = if reached_idx.is_some() {
        let to = tree.get_point(to_idx);
        let mut waypoints = vec![to.point()];
        let mut curr_point = to;

        while let Some(prev) = curr_point.connected() {
            let prev = tree.get_point(prev);
            waypoints.push(prev.point());
            curr_point = prev;
        }
        waypoints.reverse();
        Some(Path::new(waypoints, Some(to.cost())))
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
    grow_from_target: bool,
) -> Option<usize>
where
    O: Obstacle<N>,
{
    let target_point = tree.get_point(target_idx).point();

    // We first extend and rewire the tree of the other side in order to get a point that is coming
    // in the directio of `target_idx`.
    let (node, node_idx) = extend_and_rewire(target_point, tree, cfg, !grow_from_target)?;

    /*
    // Now we add many points with at most `cfg.step_size` of distance between each other in an
    // attempt to get to `target_point`.
    let mut cost = node.cost();
    let mut prev = node_idx;
    let mut new_point = node.point();
    let mut dir = target_point - new_point;

    while dir.magnitude() > cfg.step_size {
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
        cost += dir.magnitude();
        dir = target_point - new_point;
    }
    */

    let cost = node.cost();
    let mut prev = node_idx;
    let mut new_point = node.point();
    let dir = target_point - new_point;

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
    if na::distance(&target_point, &new_point) > cfg.step_size {
        return None;
    }

    // At this point, we have reached `target_point`, which means we can connect `new_point`
    // with `target_point`.

    // If `target` is grown from the target, we must connect with `other_side` the `target_point`
    // branch until the root of it's tree, in this case, the target.
    let mut curr = if grow_from_target {
        Some(target_idx)
    } else {
        // In this case `target` is grow from the origin and `prev` will point to a node in a tree
        // grown from the target. So then, we need to reverse `prev` and `curr` because we want to
        // traverse the `grow_from_target` tree.
        let curr = Some(prev);
        prev = target_idx;
        curr
    };

    // Iterate in a way that `curr.other_side` points to `prev`. And then we traverse the tree going
    // the the parent of `curr` and updating it to point to `curr`, and so on. We only traverse the
    // `grow_from_target` tree, so we make sure that `curr` is always a member of it.
    while let Some(to_update) = curr {
        let node = tree.get_point(to_update);
        let mut borrow = node.inner_mut();
        /*
        borrow.other_side = borrow.other_side
            .iter()
            .copied()
            .chain(std::iter::once(prev))
            .min_by_key(|&i| tree.get_point(i).cost().to_ord());
        */

        borrow.other_side.replace(prev);

        prev = to_update;
        curr = borrow.connected;
    }

    Some(prev)
}

///
/// NOTE: I saw some implementations where the cost values would be stored inside the node structure.
/// This would be much faster, but when rewireing there should be a situation where one of the nodes
/// may endup with a wrong cost. Consider this situation:
/// ```
/// /---------------------|
/// |      1..            |
/// |      .  ...   ...3  |
/// |     .     ...2      |
/// |    .                |
/// |    . ..P            |
/// |    O..              |
/// |---------------------|
/// ```
/// points 1, 2 and 3 were already on the tree connected with edges (dots) and a new point P was just
/// added to the graph. P will connect to O, the origin point, as it is the closest and shortest path
/// to the origin. In the update radius of P, only 2 and O will be listed. Surely, O will not be
/// updated. 2 however, does benefit from going through P instead of 1. So, when rewirering the path
/// from 2 will be 2 -> P -> O. The cost of 2 will also be updated. However, the cost of 3 will remain
/// the old cost of going 3 -> 2 -> 1 -> O and it shouldn't! The correct thing would be to also update
/// all of the nodes that go through 2. But this is a problem because it is way more convenient to
/// the tree going from the nodes to the origin, if we were to keep track of all of the incoming edges,
/// it could be a significant overhead.
///
/// Alternativelly we could call `rewire()` recursivelly on the updated node (2, in the example). This
/// would fix the problem as long as `dist(node[2], node[3])` is less than the update radius, but it
/// will also trigger lots of other rewirerings.
///
/// In the current implementation we don't trigger any other rewrite, so it's fast but may not provide
/// a very good path.
///
/// NOTE: If something goes wrong with the logic, it could cause a cycle in the graph.
///
fn rewire<'a, O, I, const N: usize>(node_idx: usize, near: I, tree: &KDTree<Node<N>, N>, cfg: &Cfg<'a, O, N>)
where
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

/*
fn calc_cost<const N: usize>(p: &na::Point<f32, N>, connected: usize, tree: &KDTree<Node<N>, N>) -> f32 {
    let mut prev = tree.get_at(connected);
    let mut cost = na::distance(&p, prev.borrow());
    while let Some(next) = prev.connected.map(|c| tree.get_at(c)) {
        cost += na::distance(prev.borrow(), next.borrow());
    }
    cost
}
*/
