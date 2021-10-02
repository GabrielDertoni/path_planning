use std::borrow::Borrow;
use std::ops::Range;

use nalgebra as na;

use crate::{ RRTResult, RRTGraph, Path };
use crate::solvers::RRTSolver;
use crate::builder::RRTBuilder;
use crate::obstacle::Obstacle;
use kd_tree::{ KDTree, HasCoords };
use crate::utils::*;
use crate::point::*;

pub struct RRTSimpleSolver;

impl<const N: usize> RRTSolver<N> for RRTSimpleSolver {
    type Node = Point<N>;

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

fn rrt_solve<O: Obstacle<N>, const N: usize>(
    from: na::Point<f32, N>,
    to: na::Point<f32, N>,
    obstacles: &[O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_steps: usize,
    target_radius: f32,
) -> RRTResult<RRTSimpleSolver, N> {
    let from = Point::new(from);
    let mut point_tree = KDTree::<Point<N>, N>::new();
    point_tree.insert(from.clone());

    let reached = 'outer: loop {
        let rnd_point = loop {
            let p = gen_random_in_range(random_range.clone());

            // NOTE: This may be a performance bottleneck. Could probably be inproved with something like
            // an AABB Tree.
            if !obstacles.iter().any(|o| o.is_inside(&p)) {
                break Point::new(p);
            }
        };

        let nearest = point_tree.find_nearest(&rnd_point)
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
                point_tree.insert(step_point.clone());
            }

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                break 'outer step_point;
            }

            i += 1;
        }
    };

    let to = Point::new_connected(to, reached);
    point_tree.insert(to.clone());

    let mut waypoints = vec![to.point()];
    let mut curr_point = to;

    while let Some(prev) = curr_point.connected().clone() {
        waypoints.push(prev.point());
        curr_point = prev;
    }
    waypoints.push(from.point());
    waypoints.reverse();

    let graph = RRTGraph::new(
        point_tree.iter()
            .cloned()
            .collect(),
        from,
    );
    
    RRTResult {
        n_points: point_tree.size(),
        graph,
        result: Some(Path::new(waypoints, None)),
    }
}