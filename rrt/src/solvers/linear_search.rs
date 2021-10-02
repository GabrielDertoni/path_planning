use std::borrow::Borrow;
use std::ops::Range;

use nalgebra as na;

use crate::obstacle::Obstacle;
// TODO: Remove this import.
use crate::{ RRTResult, RRTGraph, Path };
use crate::solvers::RRTSolver;
use kd_tree::HasCoords;
use crate::builder::RRTBuilder;
use crate::point::*;
use crate::utils::*;

pub struct RRTLinearSearchSolver;

impl<const N: usize> RRTSolver<N> for RRTLinearSearchSolver {
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
) -> RRTResult<RRTLinearSearchSolver, N> {
    let from = Point::new(from);

    let mut points = Vec::new();
    points.push(from.clone());

    let reached = 'outer: loop {
        let rnd_point = gen_random_in_range(random_range.clone());

        if obstacles.iter().any(|o| o.is_inside(&rnd_point)) {
            continue;
        }

        let nearest = points.iter()
            .min_by(|a, b| {
                let da = na::distance_squared(a.borrow(), rnd_point.borrow());
                let db = na::distance_squared(b.borrow(), rnd_point.borrow());
                da.partial_cmp(&db).unwrap()
            })
            .unwrap()
            .clone();

        // Stepping
        let mut direction: na::SVector<f32, N>  = (rnd_point - nearest.point()).cap_magnitude(step_size);
        let mut step_point = nearest;
        let mut i = 0;
        while direction.magnitude() > f32::EPSILON && i < max_steps{
            step_point = Point::new_connected(
                step_point.point() + direction,
                step_point.clone(),
            );

            // If the step point is inside any obstacle, we abandon this stepping direction.
            if obstacles.iter().any(|o| o.is_inside(step_point.borrow())) {
                break;
            } else {
                direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
                points.push(step_point.clone());
            }

            if na::distance_squared(step_point.borrow(), &to) <= target_radius * target_radius {
                break 'outer step_point;
            }

            i += 1;
        }
    };

    let to = Point::new_connected(to, reached);
    points.push(to.clone());

    let mut waypoints = vec![to.point()];
    let mut curr_point = to;

    while let Some(prev) = curr_point.connected().clone() {
        waypoints.push(prev.point());
        curr_point = prev;
    }
    waypoints.push(from.point());
    waypoints.reverse();
    
    let graph = RRTGraph::new(
        points.iter()
            .cloned()
            .collect(),
        from
    );
    
    RRTResult {
        n_points: points.len(),
        graph,
        result: Some(Path::new(waypoints, None)),
    }
}