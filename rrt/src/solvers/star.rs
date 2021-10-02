use std::borrow::Borrow;
use std::ops::Range;
use std::cell::{ RefCell, Ref, RefMut };
use std::iter::IntoIterator;

use nalgebra as na;

use kd_tree::{ KDTree, HasCoords };
use vec_array::*;
use crate::{Path, RRTGraph, RRTResult, impl_node };
use crate::solvers::RRTSolver;
use crate::builder::RRTBuilder;
use crate::obstacle::Obstacle;
use crate::utils::*;

const MAX_NEIGHBORS: usize = 10;

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
            builder.get_num_nodes(),
            0.05,
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
    neighbors: Sorted<VecArray<usize, MAX_NEIGHBORS>>,
    pub connected: Option<usize>,

}

impl<const N: usize> Node<N> {
    pub fn cost(&self) -> f32 { self.inner().borrow().cost }
    pub fn connected(&self) -> Option<usize> { self.inner().borrow().connected }
}

impl<const N: usize> Node<N> {
    fn new_root(p: na::Point<f32, N>) -> Self {
        Node {
            p,
            inner: RefCell::new(NodeInner {
                neighbors: VecArray::new_sorted(),
                cost: 0.0, connected: None
            }),
        }
    }

    fn new<I>(p: na::Point<f32, N>, cost: f32, connected: usize, neighbors: I, tree: &KDTree<Self, N>) -> Self
    where
        I: IntoIterator<Item = usize>,
    {
        let node = Node {
            p,
            inner: RefCell::new(NodeInner {
                cost,
                neighbors: VecArray::new_sorted(),
                connected: Some(connected),
            }),
        };

        for neighbor in neighbors {
            node.add_neighbor(neighbor, tree);
        }

        node
    }

    fn inner(&self) -> Ref<NodeInner> {
        self.inner.borrow()
    }

    fn inner_mut(&self) -> RefMut<NodeInner> {
        self.inner.borrow_mut()
    }

    fn connected_mut(&self) -> RefMut<Option<usize>> {
        RefMut::map(self.inner_mut(), |inner| &mut inner.connected)
    }

    fn neighbors_ref(&self) -> Ref<Sorted<VecArray<usize, MAX_NEIGHBORS>>> {
        Ref::map(self.inner(), |inner| &inner.neighbors)
    }

    fn add_neighbor(&self, neighbor: usize, tree: &KDTree<Self, N>) {
        let p = self.borrow();
        self.inner_mut().neighbors.push_evict_max_by_key(neighbor, |&el| {
            let neighbor_point = tree.get_point(el).borrow();
            OrdF32(na::distance(p, neighbor_point))
        });
    }
}

impl<const N: usize> Borrow<na::Point<f32, N>> for Node<N> {
    #[inline]
    fn borrow(&self) -> &na::Point<f32, N> { &self.p }
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
    num_nodes: usize,
    sample_goal_prob: f32,
) -> RRTResult<RRTStarSolver, N> {
    assert!(update_radius > step_size, "update_radius must be bigger than step_size");

    let from: Node<N> = Node::new_root(from).into();
    let mut point_tree: KDTree<Node<N>, N> = KDTree::new();
    point_tree.insert(from.clone());

    let mut reached_idx = None;
    let mut iters = 0;

    while iters < num_nodes {
        let rnd_point = if gen_random() > sample_goal_prob {
            gen_random_in_range(random_range.clone())
        } else {
            to
        };

        let nearest_idx = point_tree.find_index_nearest(&rnd_point).unwrap();
        let nearest = point_tree.get_point(nearest_idx);

        // Stepping
        let mut direction: na::SVector<f32, N>  = (rnd_point.point() - nearest.point()).cap_magnitude(step_size);
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
                .min_by_key(|&i| {
                    let p = point_tree.get_point(i);
                    let d = na::distance(p.borrow(), &next_step);
                    OrdF32(d + p.cost())
                })
                .unwrap_or(nearest_idx);

            let min_cost = point_tree.get_point(min_cost_idx);

            let cost = min_cost.cost() + na::distance(min_cost.borrow(), &next_step);
            step_point = Node::new(
                next_step,
                cost,
                min_cost_idx,
                within_radius.iter().cloned(),
                &point_tree,
            );
            let node_idx = point_tree.insert(step_point.clone());

            for &point in &within_radius {
                let neighbor = point_tree.get_point(point);
                neighbor.add_neighbor(node_idx, &point_tree);
            }

            rewire(node_idx, &point_tree);

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                reached_idx = reached_idx.iter()
                    .copied()
                    .chain(std::iter::once(node_idx))
                    .min_by_key(|&i| OrdF32(point_tree.get_point(i).cost()));
            }

            direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
            i += 1;
        }
        iters += 1;
    };

    let result = if let Some(reached_idx) = reached_idx {
        let reached = point_tree.get_point(reached_idx);
        let cost = reached.cost() + na::distance(reached.borrow(), &to);
        let to_idx = point_tree.insert(
            Node::new(
                to,
                cost,
                reached_idx,
                std::iter::empty(),
                &point_tree,
            )
        );

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

    let graph = RRTGraph::new(
        point_tree.iter()
            .cloned()
            .collect(),
        from,
    );
    
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
fn rewire<const N: usize>(node_idx: usize, tree: &KDTree<Node<N>, N>) {
    let node = tree.get_point(node_idx);
    // If we find a cycle, the call to `neighbors_ref` will panic.
    let near = node.neighbors_ref();
    for &neighbor_idx in near.iter() {
        let neighbor = tree.get_point(neighbor_idx);
        let old_to_new = na::distance(neighbor.borrow(), node.borrow());
        if old_to_new + node.cost() < neighbor.cost() {
            neighbor.connected_mut().replace(node_idx);
            rewire(neighbor_idx, tree);
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
