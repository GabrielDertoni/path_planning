use std::borrow::Borrow;
use std::marker::PhantomData;
use std::ops::Range;

use nalgebra as na;

use crate::{ RRTResult, RRTGraph };
use crate::obstacle::Obstacle;
use crate::utils::*;
use crate::solvers::*;

pub struct RRTBuilder<S: RRTSolver<N>, O, const N: usize> {
    from: PointN<N>,
    to: PointN<N>,
    graph: Option<RRTGraph<S, N>>,
    obstacles: Vec<O>,
    step_size: Option<f32>,
    max_steps: usize,
    target_radius: Option<f32>,
    update_radius: Option<f32>,
    random_range: Range<Option<PointN<N>>>,
    max_iters: Option<usize>,
    sample_goal_prob: Option<f32>,
    _solver: PhantomData<S>,
}

impl<S, O, const N: usize> RRTBuilder<S, O, N>
where
    O: Obstacle<N>,
    S: RRTSolver<N>,
{
    pub fn new(from: PointN<N>, to: PointN<N>) -> RRTBuilder<S, O, N> {
        RRTBuilder {
            from,
            to,
            graph: None,
            obstacles: Vec::new(),
            step_size: None,
            max_steps: 5,
            target_radius: None,
            update_radius: None,
            random_range: Range {
                start: None,
                end: None,
            },
            max_iters: None,
            sample_goal_prob: None,
            _solver: PhantomData,
        }
    }

    pub fn with_graph(mut self, graph: RRTGraph<S, N>) -> Result<Self, DifferentOriginsError> {
        if na::distance(&self.from, graph.origin.borrow()) > self.get_target_radius() {
            Err(DifferentOriginsError)
        } else {
            self.graph.replace(graph);
            Ok(self)
        }
    }

    pub fn add_obstacle(mut self, obstacle: O) -> Self {
        self.obstacles.push(obstacle);
        self
    }

    pub fn extend_obstacles(mut self, obstacles: impl IntoIterator<Item = O>) -> Self {
        self.obstacles.extend(obstacles);
        self
    }

    pub fn with_step_size(mut self, step_size: f32) -> Self {
        self.step_size.replace(step_size);
        self
    }

    pub fn with_max_stepping(mut self, max_steps: usize) -> Self {
        self.max_steps = max_steps;
        self
    }

    pub fn with_target_radius(mut self, target_radius: f32) -> Self {
        self.target_radius.replace(target_radius);
        self
    }

    pub fn with_random_range_end(mut self, end: PointN<N>) -> Self {
        self.random_range.end.replace(end);
        self
    }

    pub fn with_random_range_start(mut self, start: PointN<N>) -> Self {
        self.random_range.start.replace(start);
        self
    }

    pub fn with_random_range(self, random_range: Range<PointN<N>>) -> Self {
        self
            .with_random_range_start(random_range.start)
            .with_random_range_end(random_range.end)
    }

    pub fn with_max_iters(mut self, num_nodes: usize) -> Self {
        self.max_iters.replace(num_nodes);
        self
    }

    pub fn with_sample_goal_prob(mut self, prob: f32) -> Self {
        assert!(prob >= 0.0 && prob <= 1.0, "probability must be between 0 and 1, but was {}", prob);
        self.sample_goal_prob.replace(prob);
        self
    }

    pub fn with_update_radius(mut self, radius: f32) -> Self {
        self.update_radius.replace(radius);
        self
    }

    pub fn solve(self) -> RRTResult<S, N> {
        S::rrt_solve(self)
    }

    pub fn get_from(&self) -> PointN<N> {
        self.from
    }

    pub fn get_to(&self) -> PointN<N> {
        self.to
    }

    pub fn get_obstacles(&self) -> &[O] {
        self.obstacles.as_slice()
    }

    pub fn get_step_size(&self) -> f32 {
        self.step_size
            .unwrap_or(na::distance(&self.from, &self.to) / 100.0)
    }

    pub fn get_max_steps(&self) -> usize {
        self.max_steps
    }

    pub fn get_target_radius(&self) -> f32 {
        let step_size = self.get_step_size();
        self.target_radius
            .unwrap_or(step_size / 2.0)
    }

    pub fn get_random_range_start(&self) -> PointN<N> {
        self.random_range.start
            .unwrap_or_else(|| {
                let target_radius = self.get_target_radius();
                // In case there is no random range setup. Choose a range that encapsulates the source obstacle
                // and the destination and leaves some padding.
                let min = na::SVector::<f32, N>::from_iterator(
                    self.from.iter()
                        .zip(self.to.iter())
                        .map(|(&one, &other)| one.min(other))
                );
                PointN::from(min - na::SVector::repeat(target_radius))
            })
    }

    pub fn get_random_range_end(&self) -> PointN<N> {
        self.random_range.end
            .unwrap_or_else(|| {
                let target_radius = self.get_target_radius();
                let max = na::SVector::<f32, N>::from_iterator(
                    self.from.iter()
                        .zip(self.to.iter())
                        .map(|(&one, &other)| one.max(other)),
                );
                PointN::from(max + na::SVector::repeat(target_radius))
            })
    }

    pub fn get_random_range(&self) -> Range<PointN<N>> {
        Range {
            start: self.get_random_range_start(),
            end: self.get_random_range_end(),
        }
    }

    pub fn get_max_iters(&self) -> usize {
        self.max_iters
            .unwrap_or_else(|| {
                let range = self.get_random_range();
                let step_size = self.get_step_size();
                range.start.iter()
                    .zip(range.end.iter())
                    .map(|(&s, &e)| ((e - s) / step_size) as usize)
                    .product()
            })
    }

    pub fn get_sample_goal_prob(&self) -> f32 {
        self.sample_goal_prob.unwrap_or(0.1)
    }

    pub fn get_update_radius(&self) -> f32 {
        self.update_radius.unwrap_or(self.get_step_size() * 2.0)
    }
}

impl<O, const N: usize> RRTBuilder<RRTSimpleSolver, O, N>
where
    O: Obstacle<N>,
{
    #[inline(always)]
    pub fn new_simple(from: PointN<N>, to: PointN<N>) -> RRTBuilder<RRTSimpleSolver, O, N> {
        RRTBuilder::new(from, to)
    }

    /// This function is usefull for type inference. It is a no-op, but it indicates to the
    /// compiler that this builder has to have solver type `RRTSimpleSolver`.
    #[inline(always)]
    pub fn as_simple(self) -> Self { self }
}

impl<S: RRTSolver<N>, const N: usize> RRTBuilder<S, Box<dyn Obstacle<N>>, N> {
    /// This function is usefull for type inference. It is a no-op, but it indicates to the
    /// compiler that this builder has to have obstacle type `Box<dyn Obstacle<N>>`.
    #[inline(always)]
    pub fn with_dyn_obstacles(self) -> Self { self }
}

impl<'a, S: RRTSolver<N>, const N: usize> RRTBuilder<S, &'a dyn Obstacle<N>, N> {
    /// This function is usefull for type inference. It is a no-op, but it indicates to the
    /// compiler that this builder has to have obstacle type `&dyn Obstacle<N>`.
    #[inline(always)]
    pub fn with_dyn_ref_obstacles(self) -> Self { self }
}

use std::fmt::{ self, Debug, Display };
use std::error::Error;

pub struct DifferentOriginsError;

impl Display for DifferentOriginsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "when reusing an RRT Graph it is necessary to use the same origin point")
    }
}

impl Debug for DifferentOriginsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(self, f)
    }
}

impl Error for DifferentOriginsError {}