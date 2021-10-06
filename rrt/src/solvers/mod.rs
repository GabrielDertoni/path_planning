pub mod simple;
pub mod linear_search;
#[cfg(not(feature = "rrt_start_innacurate_solver"))]
pub mod star;
#[cfg(feature = "rrt_start_innacurate_solver")]
pub mod star_inaccurate;

pub mod star_connect;

#[cfg(feature = "multithreaded")]
pub mod multithreaded;

pub use simple::RRTSimpleSolver;
pub use linear_search::RRTLinearSearchSolver;

#[cfg(not(feature = "rrt_start_innacurate_solver"))]
pub use star::RRTStarSolver;
#[cfg(feature = "rrt_start_innacurate_solver")]
pub use star_inaccurate::RRTStarSolver;

pub use star_connect::RRTStarConnectSolver;

#[cfg(feature = "multithreaded")]
pub use multithreaded::RRTMultithreadedSolver;

#[macro_use]
mod macros;

use std::borrow::Borrow;

use nalgebra as na;

use crate::RRTResult;
use crate::builder::RRTBuilder;
use crate::obstacle::Obstacle;

pub trait RRTSolver<const N: usize>: Sized {
    type Node: Borrow<na::Point<f32, N>>;
    fn rrt_solve<O: Obstacle<N>>(builder: RRTBuilder<Self, O, N>) -> RRTResult<Self, N>;
}
