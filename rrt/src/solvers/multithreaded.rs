#![allow(dead_code, unused_imports)]

use std::sync::{ Arc, RwLock };
use std::borrow::Borrow;
use std::ops::Range;

use nalgebra as na;

use crate::{ RRTResult, RRTGraph, Path };
use crate::solvers::RRTSolver;
use crate::obstacle::Obstacle;
use crate::builder::RRTBuilder;
use crate::kd_tree::{ KDTree, HasCoords };
use crate::utils::*;
use crate::point::*;

pub struct RRTMultithreadedSolver;

impl<const N: usize> RRTSolver<N> for RRTMultithreadedSolver {
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
        )
    }
}

pub struct Node<const N: usize>(Arc<NodeData<N>>);
pub struct NodeData<const N: usize> {
    p: na::Point<f32, N>,
    connected: Option<Node<N>>,
}

impl<const N: usize> Borrow<na::Point<f32, N>> for Node<N> {
    fn borrow(&self) -> &na::Point<f32, N> {
        &self.0.p
    }
}

// TODO: make multithreaded.
fn rrt_solve<O: Obstacle<N>, const N: usize>(
    from: na::Point<f32, N>,
    to: na::Point<f32, N>,
    obstacles: &[O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_steps: usize,
    target_radius: f32,
) -> RRTResult<RRTMultithreadedSolver, N> {
    todo!()
    /*
    let from = Point::new(from);
    let point_tree = Arc::new(RwLock::new(KDTree::<Point<N>, N>::new()));
    point_tree.write()
        .unwrap()
        .insert(from.clone());

    let reached = 'outer: loop {
        let rnd_point = loop {
            let p = gen_random_in_range(random_range.clone());

            // NOTE: This may be a performance bottleneck. Could probably be inproved with something like
            // an AABB Tree.
            if !obstacles.iter().any(|o| o.is_inside(&p)) {
                break Point::new(p);
            }
        };

        let nearest = point_tree.read()
            .unwrap()
            .find_nearest(&rnd_point)
            .unwrap()
            .clone();

        // Stepping
        let mut direction: na::SVector<f32, N>  = (rnd_point.point() - nearest.point()).cap_magnitude(step_size);
        let mut step_point = nearest;
        let mut i = 0;
        while direction.magnitude() > f32::EPSILON && i < max_steps {
            step_point = Point::new_connected(
                step_point.point() + direction,
                step_point.clone(),
            );

            // If the step point is inside any obstacle, we abandon this stepping direction.
            if obstacles.iter().any(|o| o.is_inside(step_point.borrow())) {
                break;
            } else {
                direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
                point_tree.write()
                    .unwrap()
                    .insert(step_point.clone());
            }

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                break 'outer step_point;
            }

            i += 1;
        }
    };

    let to = Point::new_connected(to, reached);
    point_tree.write()
        .unwrap()
        .insert(to.clone());

    let mut waypoints = vec![to.point()];
    let mut curr_point = to;

    while let Some(prev) = curr_point.connected().clone() {
        waypoints.push(prev.point());
        curr_point = prev;
    }
    waypoints.push(from.point());
    waypoints.reverse();

    let read_lock = point_tree.read().unwrap();

    let graph = RRTGraph::new( 
        read_lock.iter()
            .cloned()
            .collect(),
        from,
     );
    
    RRTResult {
        n_points: read_lock.size(),
        graph,
        result: Some(Path::new(waypoints)),
    }
    */
}