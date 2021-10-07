#![feature(slice_index_methods)]

pub mod obstacle;
pub mod point;
pub mod solvers;
pub mod builder;

mod utils;

pub use builder::*;
pub use solvers::*;

use nalgebra as na;

// TODO: Implement Iterator<Item = na::Point<f32, N>> for Path<N> and hide the waypoints
// field. This would allow Path to actually hold a vec of Point<N> which could be better
// used in other places of the RRT.
pub struct Path<const N: usize> {
    pub waypoints: Vec<na::Point<f32, N>>,
    pub cost: Option<f32>,
}

pub struct RRTResult<S: RRTSolver<N>, const N: usize> {
    pub n_points: usize,
    pub graph: RRTGraph<S, N>,
    pub result: Option<Path<N>>,
}

impl<const N: usize> Path<N> {
    pub fn new(waypoints: Vec<na::Point<f32, N>>, cost: Option<f32>) -> Self {
        Path { waypoints, cost }
    }
}

pub struct RRTGraph<S: RRTSolver<N>, const N: usize> {
    pub points: Vec<S::Node>,
    origin: S::Node,
}

impl<S, const N: usize> Clone for RRTGraph<S, N>
where
    S: RRTSolver<N>,
    S::Node: Clone,
{
    fn clone(&self) -> Self {
        RRTGraph {
            points: self.points.clone(),
            origin: self.origin.clone(),
        }
    }
}

impl<S: RRTSolver<N>, const N: usize> RRTGraph<S, N> {
    fn new(points: Vec<S::Node>, origin: S::Node) -> Self {
        RRTGraph { points, origin }
    }
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