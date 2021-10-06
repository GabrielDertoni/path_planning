
pub trait Space: Default {
    type Point;

    /// Sample a valid point in the space. This point must be valid and random.
    fn sample(&self) -> Self::Point;

    fn distance(&self, p1: &Self::Point, p2: &Self::Point) -> f32;

    fn min() -> Self::Point;
    fn max() -> Self::Point;
}

pub trait Environment: Space {
    /// Verify if the entity is in a valid position.
    fn is_valid(&self, point: &Self::Point) -> bool;
}

pub trait Entity<S: Environment> {
    fn steer(&self, towards: &S::Point) -> S::Point;
}