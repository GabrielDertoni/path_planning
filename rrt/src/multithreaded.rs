use std::ops::Range;
use std::borrow::Borrow;

use nalgebra as na;

use crate::obstacle::Obstacle;
use crate::kd_tree::{HasCoords, KDTree};
use crate::point::*;
use crate::utils::*;

use super::Path;

pub fn rrt_solve_multithreadded<O, const N: usize>(
    source: na::Point<f32, N>,
    destination: na::Point<f32, N>,
    obstacles: &[O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_steps: usize,
    target_radius: f32,
) -> Path<N>
where
    O: AsRef<dyn Obstacle<N>>,
{
    let source = Point::new(source);
    let mut point_tree = KDTree::<Point<N>, N>::new();
    point_tree.insert(source.clone());

    let reached = 'outer: loop {
        let rnd_point = gen_random_in_range(random_range.clone());

        if obstacles.iter().any(|o| o.as_ref().is_inside(&rnd_point)) {
            continue;
        }

        let nearest = point_tree.find_nearest(&rnd_point)
            .unwrap()
            .clone();

        // Stepping
        let mut direction: na::SVector<f32, N>  = (rnd_point - nearest.point()).cap_magnitude(step_size);
        let mut step_point = nearest;
        let mut i = 0;
        while direction.magnitude() > f32::EPSILON && i < max_steps {
            step_point = Point::new_connected(
                step_point.point() + direction,
                step_point.clone(),
            );

            // If the step point is inside any obstacle, we abandon this stepping direction.
            if obstacles.iter().any(|o| o.as_ref().is_inside(step_point.borrow())) {
                break;
            } else {
                direction = (rnd_point.point() - step_point.point()).cap_magnitude(step_size);
                point_tree.insert(step_point.clone());
            }

            if na::distance_squared(step_point.borrow(), &destination) <= target_radius * target_radius {
                break 'outer step_point;
            }

            i += 1;
        }
    };

    let mut waypoints = vec![reached.point()];
    let mut curr_point = reached;

    while let Some(prev) = curr_point.connected().clone() {
        waypoints.push(prev.point());
        curr_point = prev;
    }
    waypoints.push(source.point());
    waypoints.reverse();
    
    Path::new(waypoints)
}