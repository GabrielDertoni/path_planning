use std::borrow::Borrow;
use std::collections::VecDeque;
use std::iter::IntoIterator;
use std::ops::Range;

use lazy_static::lazy_static;
use nalgebra as na;
use statrs::distribution::{ContinuousCDF, Normal};

use kd_tree::{
    visitor::{NearestIndexedVisitor, PreconditionVisitor, WithinRadiusIndicesVisitor},
    HasCoords, KDTree,
};

use super::node::*;
use crate::obstacle::{Convex, FaceNormals};
use crate::utils::*;
use crate::Path;

/*
pub struct RRTRiskAlloc2DSolver;

impl RRTSolver<2> for RRTRiskAlloc2DSolver {
    type Node = Node<2>;

    fn rrt_solve<O: Obstacle<2>>(builder: RRTBuilder<Self, O, 2>) -> RRTResult<Self, 2> {
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
*/

pub fn rrt_solve<O, const N: usize>(
    from: na::Point<f32, N>,
    to: na::Point<f32, N>,
    obstacles: &[O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_stepping: usize,
    target_radius: f32,
    update_radius: f32,
    max_iters: usize,
    sample_goal_prob: f32,
    risk: f32,
    uncertainty: f32,
) -> (Option<Path<N>>, Vec<Node<N>>)
where
    O: FaceNormals<N> + Convex<N>,
{
    assert!(
        update_radius >= step_size,
        "update_radius must be bigger than step_size"
    );

    let from: Node<N> = Node::new_root(from).into();
    let mut tree: KDTree<Node<N>, N> = KDTree::new();
    tree.insert(from.clone());

    let mut reached_idx = None;
    let mut iters = 0;

    let margin = safe_margin(risk, uncertainty);

    while iters < max_iters {
        let rnd_point = if gen_random() > sample_goal_prob {
            gen_random_in_range(random_range.clone())
        } else {
            to
        };

        let vis = NearestIndexedVisitor::new();
        let (nearest_idx, nearest) = tree.query(&rnd_point, vis).unwrap();

        // Stepping
        let mut direction: na::SVector<f32, N> =
            (rnd_point.point() - nearest.point()).cap_magnitude(step_size);
        let mut step_point = nearest.clone();
        let mut i = 0;
        while direction.magnitude() > f32::EPSILON && i < max_stepping {
            let next_step = step_point.point() + direction;

            // If any of the distances is smaller than our safety margin, it's not a valid point.
            if obstacles
                .iter()
                .any(|o| estimate_dist(&next_step, o) < margin)
            {
                break;
            }

            let vis = PreconditionVisitor::new(
                WithinRadiusIndicesVisitor::new(update_radius),
                |p: &Node<N>| {
                    // Filter out points that would have an obstructed path to the newly added point.
                    obstacles.iter().all(|o| {
                        estimate_path_is_safe(p.borrow(), &next_step, o, risk, uncertainty)
                    })
                },
            );
            // let vis = WithinRadiusIndicesVisitor::new(update_radius);
            let within_radius = tree.query(&next_step, vis);

            // Find the minimum cost point P such that dist(P, next_step) + cost(P) is minimized.
            let min_cost_idx = within_radius
                .iter()
                .copied()
                .min_by_key(|&i| {
                    let p = tree.get_point(i);
                    let d = na::distance(p.borrow(), &next_step);
                    (d + p.cost()).to_ord()
                })
                .unwrap_or(nearest_idx);

            let min_cost = tree.get_point(min_cost_idx);

            let cost = min_cost.cost() + na::distance(min_cost.borrow(), &next_step);
            step_point = Node::new(next_step, cost, min_cost_idx, std::iter::empty());
            let node_idx = tree.insert(step_point.clone());

            tree.get_point(min_cost_idx).add_child(node_idx);

            rewire(node_idx, within_radius, &tree, step_size);

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                reached_idx = reached_idx
                    .iter()
                    .copied()
                    .chain(std::iter::once(node_idx))
                    .min_by_key(|&i| tree.get_point(i).cost().to_ord());
            }

            direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
            i += 1;
        }
        iters += 1;
    }

    let reached_idx = if let Some(val) = reached_idx {
        val
    } else {
        return (None, tree.iter().cloned().collect());
    };

    let reached = tree.get_point(reached_idx);
    let cost = reached.cost() + na::distance(reached.borrow(), &to);
    let to_idx = tree.insert(Node::new(to, cost, reached_idx, std::iter::empty()));

    let to = tree.get_point(to_idx);

    let mut waypoints = vec![to.point()];
    let mut curr_point = to;

    while let Some(prev) = curr_point.connected() {
        let prev = tree.get_point(prev);
        waypoints.push(prev.point());
        curr_point = prev;
    }
    waypoints.push(from.point());
    waypoints.reverse();

    let path = Path::new(waypoints, Some(to.cost()));
    (Some(path), tree.iter().cloned().collect())
}

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

lazy_static! {
    static ref NORM: Normal = Normal::new(0.0, 1.0).unwrap();
}

/// Estimates the distance from a point to an obstacle. This function will always return a
/// distance that is less then or equal to the actual distance to the obstacle.
fn estimate_dist<O, const N: usize>(point: &na::Point<f32, N>, obstacle: &O) -> f32
where
    O: FaceNormals<N> + Convex<N>,
{
    obstacle
        .face_points()
        .iter()
        .zip(obstacle.face_normals().iter())
        .map(|(p, n)| (n.dot(&p.coords), n))
        // This forms the equation of a hyperplane: n * x = b.
        .map(|(b, n)| (n.dot(&point.coords) - b).to_ord())
        .max()
        .unwrap()
        .from_ord()
}

/// Returns the probability of collision (risk) of colliding with an obstacle at distance `dist`. The
/// value of `dist` is assumed to have `uncertainty`.
fn prob_collision(dist: f32, uncertainty: f32) -> f32 {
    if dist <= 0.0 {
        1.0
    } else {
        1.0 - NORM.cdf((dist / uncertainty) as f64) as f32
    }
}

/// Returns the minimum distance to keep away from obstacles in order to mantain a probability of
/// collision at most `risk` and with `uncertainty`.
fn safe_margin(risk: f32, uncertainty: f32) -> f32 {
    uncertainty * NORM.inverse_cdf((1.0 - risk) as f64) as f32
}

/// This function will return wether it is safe to assume that there are no obstacles intersecting
/// with the segment from `p1` to `p2`. If the function returns `true` for sure there is a no
/// obstacle intersecting the segment, and the path will be safe according to the `risk` and
/// `uncertainty` variables. However, it is possible that the function will invalidate paths that
/// would also be safe. Therefore, it is good to choose `p1` and `p2` relatively near each other.
///
/// # Algorithm idea
///
/// If both points are in the same side of a convex shape and outside of it, then the segment from
/// `p1` to `p2` will for sure not intersect the shape (obstacle).
fn estimate_path_is_safe<O, const N: usize>(
    p1: &na::Point<f32, N>,
    p2: &na::Point<f32, N>,
    obstacle: &O,
    risk: f32,
    uncertainty: f32,
) -> bool
where
    O: FaceNormals<N> + Convex<N>,
{
    obstacle
        .face_points()
        .iter()
        .zip(obstacle.face_normals().iter())
        .map(|(p, n)| (n.dot(&p.coords), n))
        .any(|(b, n)| {
            // Distance from p1 to edge
            let d1 = n.dot(&p1.coords) - b;
            // Distance from p2 to edge
            let d2 = n.dot(&p2.coords) - b;

            // Calculate the probability of collision with both of those distances
            let delta1 = prob_collision(d1, uncertainty);
            let delta2 = prob_collision(d2, uncertainty);
            delta1 < risk && delta2 < risk
        })
}
