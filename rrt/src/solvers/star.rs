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
use kd_tree::{HasCoords, KDTree, visitor::NearestIndexedVisitor};

/// An RRT* solver that is fast, but uses more memory then it's counterparts. This drawback
/// is because each node has to have a reference to each of it's children in a dynamically
/// allocated vector.
pub struct RRTStarSolver;

impl<const N: usize> RRTSolver<N> for RRTStarSolver {
    type Node = Node<N>;

    fn rrt_solve<O: Obstacle<N>>(builder: RRTBuilder<Self, O, N>) -> RRTResult<Self, N> {
        rrt_solve(
            builder.get_from(),
            builder.get_to(),
            builder.get_obstacles(),
            builder.get_random_range(),
            builder.get_step_size(),
            builder.get_max_steps(),
            builder.get_target_radius(),
            builder.get_update_radius(),
            builder.get_max_iters(),
            builder.get_sample_goal_prob(),
        )
    }
}

#[derive(Clone, Debug)]
pub struct Node<const N: usize> {
    p: na::Point<f32, N>,
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
    pub fn children(&self) -> Ref<'_, [usize]> {
        Ref::map(self.inner(), |inner| inner.children.as_slice())
    }
}

impl<const N: usize> Node<N> {
    fn new_root(p: na::Point<f32, N>) -> Self {
        Node {
            p,
            inner: RefCell::new(NodeInner {
                children: Vec::new(),
                cost: 0.0,
                connected: None,
            }),
        }
    }

    fn new<I>(p: na::Point<f32, N>, cost: f32, connected: usize, children: I) -> Self
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

fn rrt_solve<O: Obstacle<N>, const N: usize>(
    from: na::Point<f32, N>,
    to: na::Point<f32, N>,
    obstacles: &[O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_steps: usize,
    target_radius: f32,
    update_radius: f32,
    max_iters: usize,
    sample_goal_prob: f32,
) -> RRTResult<RRTStarSolver, N> {
    assert!(
        update_radius >= step_size,
        "update_radius must be bigger than step_size"
    );

    let from: Node<N> = Node::new_root(from);
    let mut point_tree: KDTree<Node<N>, N> = KDTree::new();
    point_tree.insert(from.clone());

    let mut reached_idx = None;
    let mut iters = 0;

    while iters < max_iters {
        let rnd_point = if gen_random() > sample_goal_prob {
            gen_random_in_range(random_range.clone())
        } else {
            to
        };

        let vis = NearestIndexedVisitor::new();
        let (nearest_idx, nearest) = point_tree.query(&rnd_point, vis).unwrap();

        // Stepping
        let mut direction: na::SVector<f32, N> =
            (rnd_point.point() - nearest.point()).cap_magnitude(step_size);
        let mut step_point = nearest.clone();
        let mut i = 0;
        while direction.magnitude() > f32::EPSILON && i < max_steps {
            let next_step = step_point.point() + direction;

            // If the step point is inside any obstacle, we abandon this stepping direction.
            // NOTE: This may be a performance bottleneck. Could probably be inproved with something like
            // an AABB Tree.
            if obstacles.iter().any(|o| o.is_inside(&next_step)) {
                break;
            }

            let within_radius = point_tree.find_indices_within_radius(&next_step, update_radius);

            // Find the minimum cost point P such that dist(P, next_step) + cost(P) is minimized.
            let min_cost_idx = within_radius
                .iter()
                .copied()
                .filter(|&i| {
                    let p = point_tree.get_point(i);
                    let d = na::distance(p.borrow(), &next_step);
                    d <= step_size
                })
                .min_by_key(|&i| {
                    let p = point_tree.get_point(i);
                    let d = na::distance(p.borrow(), &next_step);
                    (d + p.cost()).to_ord()
                })
                .unwrap_or(nearest_idx);

            let min_cost = point_tree.get_point(min_cost_idx);

            let cost = min_cost.cost() + na::distance(min_cost.borrow(), &next_step);
            step_point = Node::new(next_step, cost, min_cost_idx, std::iter::empty());
            let node_idx = point_tree.insert(step_point.clone());

            point_tree.get_point(min_cost_idx).add_child(node_idx);

            rewire(node_idx, within_radius, &point_tree, step_size);

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                reached_idx = reached_idx
                    .iter()
                    .copied()
                    .chain(std::iter::once(node_idx))
                    .min_by_key(|&i| point_tree.get_point(i).cost().to_ord());
            }

            direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
            i += 1;
        }
        iters += 1;
    }

    let result = if let Some(reached_idx) = reached_idx {
        let reached = point_tree.get_point(reached_idx);
        let cost = reached.cost() + na::distance(reached.borrow(), &to);
        let to_idx = point_tree.insert(Node::new(to, cost, reached_idx, std::iter::empty()));

        let to = point_tree.get_point(to_idx);

        let mut waypoints = vec![to.point()];
        let mut curr_point = to;

        while let Some(prev) = curr_point.connected() {
            let prev = point_tree.get_point(prev);
            waypoints.push(prev.point());
            curr_point = prev;
        }
        waypoints.push(from.point());
        waypoints.reverse();
        Some(Path::new(waypoints, Some(to.cost())))
    } else {
        None
    };

    let graph = RRTGraph::new(point_tree.iter().cloned().collect(), from);
    RRTResult {
        n_points: point_tree.size(),
        graph,
        result,
    }
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
fn rewire<I, const N: usize>(node_idx: usize, near: I, tree: &KDTree<Node<N>, N>, step_size: f32)
where
    I: IntoIterator<Item = usize>,
{
    let node = tree.get_point(node_idx);
    for neighbor_idx in near.into_iter() {
        let neighbor = tree.get_point(neighbor_idx);
        let old_to_new = na::distance(neighbor.borrow(), node.borrow());
        if old_to_new <= step_size && neighbor.cost() > old_to_new + node.cost() {
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

#[derive(Clone)]
pub struct RRTStarIter<O, const N: usize> {
    pub from: Node<N>,
    pub to: na::Point<f32, N>,
    pub obstacles: Vec<O>,
    pub random_range: Range<na::Point<f32, N>>,
    pub step_size: f32,
    pub max_steps: usize,
    pub target_radius: f32,
    pub update_radius: f32,
    pub max_iters: usize,
    pub sample_goal_prob: f32,
    pub kd_tree: KDTree<Node<N>, N>,
    pub iters: usize,
    pub reached_idx: Option<usize>,
}

impl<O: Obstacle<N>, const N: usize> RRTStarIter<O, N> {
    pub fn new(
        from: na::Point<f32, N>,
        to: na::Point<f32, N>,
        obstacles: Vec<O>,
        random_range: Range<na::Point<f32, N>>,
        step_size: f32,
        max_steps: usize,
        target_radius: f32,
        update_radius: f32,
        max_iters: usize,
        sample_goal_prob: f32,
    ) -> Self {
        assert!(
            update_radius >= step_size,
            "update_radius must be bigger than step_size"
        );

        let mut kd_tree = KDTree::new();
        let from: Node<N> = Node::new_root(from);
        kd_tree.insert(from.clone());

        RRTStarIter {
            from,
            to,
            obstacles,
            random_range,
            step_size,
            max_steps,
            target_radius,
            update_radius,
            max_iters,
            sample_goal_prob,
            kd_tree,
            iters: 0,
            reached_idx: None,
        }
    }

    pub fn from_builder(mut builder: RRTBuilder<RRTStarSolver, O, N>) -> Self {
        Self::new(
            builder.get_from(),
            builder.get_to(),
            std::mem::take(&mut builder.obstacles),
            builder.get_random_range(),
            builder.get_step_size(),
            builder.get_max_steps(),
            builder.get_target_radius(),
            builder.get_update_radius(),
            builder.get_max_iters(),
            builder.get_sample_goal_prob(),
        )
    }
}

impl<O: Obstacle<N>, const N: usize> Iterator for RRTStarIter<O, N> {
    type Item = ();

    fn next(&mut self) -> Option<()> {
        let &mut RRTStarIter {
            to,
            ref mut obstacles,
            ref mut random_range,
            step_size,
            max_steps,
            target_radius,
            update_radius,
            max_iters,
            sample_goal_prob,
            ref mut kd_tree,
            ref mut iters,
            ref mut reached_idx,
            ..
        } = self;

        if *iters >= max_iters {
            return None;
        }
        let rnd_point = if gen_random() > sample_goal_prob {
            gen_random_in_range(random_range.clone())
        } else {
            to
        };

        let vis = NearestIndexedVisitor::new();
        let (nearest_idx, nearest) = kd_tree.query(&rnd_point, vis).unwrap();

        // Stepping
        let mut direction: na::SVector<f32, N> =
            (rnd_point.point() - nearest.point()).cap_magnitude(step_size);
        let mut step_point = nearest.clone();
        let mut i = 0;
        while direction.magnitude() > f32::EPSILON && i < max_steps {
            let next_step = step_point.point() + direction;

            // If the step point is inside any obstacle, we abandon this stepping direction.
            // NOTE: This may be a performance bottleneck. Could probably be inproved with something like
            // an AABB Tree.
            if obstacles.iter().any(|o| o.is_inside(&next_step)) {
                break;
            }

            let within_radius = kd_tree.find_indices_within_radius(&next_step, update_radius);

            // Find the minimum cost point P such that dist(P, next_step) + cost(P) is minimized.
            let min_cost_idx = within_radius
                .iter()
                .copied()
                .filter(|&i| {
                    let p = kd_tree.get_point(i);
                    let d = na::distance(p.borrow(), &next_step);
                    d <= step_size
                })
                .min_by_key(|&i| {
                    let p = kd_tree.get_point(i);
                    let d = na::distance(p.borrow(), &next_step);
                    (d + p.cost()).to_ord()
                })
                .unwrap_or(nearest_idx);

            let min_cost = kd_tree.get_point(min_cost_idx);

            let cost = min_cost.cost() + na::distance(min_cost.borrow(), &next_step);
            step_point = Node::new(next_step, cost, min_cost_idx, std::iter::empty());
            let node_idx = kd_tree.insert(step_point.clone());

            kd_tree.get_point(min_cost_idx).add_child(node_idx);

            rewire(node_idx, within_radius, kd_tree, step_size);

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                *reached_idx = reached_idx
                    .iter()
                    .copied()
                    .chain(std::iter::once(node_idx))
                    .min_by_key(|&i| kd_tree.get_point(i).cost().to_ord());
            }

            direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
            i += 1;
        }
        *iters += 1;

        Some(())
    }
}
