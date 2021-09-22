pub mod obstacle;
pub mod kd_tree;
// TODO: Should be only used when feature "multithreading" is enabled
pub mod multithreaded;
pub mod point;

mod utils;

pub use multithreaded::*;

use std::ops::Range;
use std::borrow::Borrow;

use nalgebra as na;

use obstacle::Obstacle;
use kd_tree::{HasCoords, KDTree};
use point::*;
use utils::*;

pub struct Path<const N: usize> {
    pub waypoints: Vec<na::Point<f32, N>>
}

impl<const N: usize> Path<N> {
    pub fn new(waypoints: Vec<na::Point<f32, N>>) -> Self { Self { waypoints } }
}

pub fn rrt_solve<O, const N: usize>(
    source: na::Point<f32, N>,
    destination: na::Point<f32, N>,
    obstacles: &[O],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_steps: usize,
    target_radius: f32,
    point_pool_size: usize,
) -> Path<N>
where
    O: AsRef<dyn Obstacle<N>>,
{
    let source = Point::new(source);
    let mut point_tree = KDTree::<Point<N>, N>::new();
    point_tree.insert(source.clone());

    let reached = 'outer: loop {
        let mut point_pool = Vec::with_capacity(point_pool_size);

        for _ in 0..point_pool_size {
            let rnd_point = loop {
                let p = gen_random_in_range(random_range.clone());

                // NOTE: This may be a performance bottleneck. Could probably be inproved with something like
                // an AABB Tree.
                if !obstacles.iter().any(|o| o.as_ref().is_inside(&p)) {
                    break Point::new(p);
                }
            };

            point_pool.push(rnd_point);
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

pub fn rrt_solve_points_vec<const N: usize>(
    source: na::Point<f32, N>,
    destination: na::Point<f32, N>,
    obstacles: &[Box<dyn Obstacle<N>>],
    random_range: Range<na::Point<f32, N>>,
    step_size: f32,
    max_steps: usize,
    target_radius: f32,
) -> Path<N> {
    let source = Point::new(source);

    let mut points = Vec::new();
    points.push(source.clone());

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

#[cfg(test)]
mod tests {
    /*
    use super::*;
    use nalgebra as na;
    use crate::obstacle::*;

    #[test]
    fn spheres() {
        let obstacle1 = Sphere::new(na::Point2::new(1.0, 0.0), 0.5);
        let obstacle2 = Sphere::new(na::Point2::new(2.0, 0.0), 0.5);
        let target = na::Point2::new(3.0, 0.0);
        let origin = na::Point2::new(0.0, 0.0);

        let path = rrt_solve(
            origin,
            target,
            &[Box::new(obstacle1), Box::new(obstacle2)],
            Range { start: na::Point2::new(-1.0, -1.0), end: na::Point2::new(4.0, 1.0) },
            0.2,
            0.1,
        );
    }
    */
}