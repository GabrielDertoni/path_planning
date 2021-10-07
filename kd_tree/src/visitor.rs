use std::borrow::Borrow;
use std::marker::PhantomData;

use nalgebra as na;

use crate::HasCoords;

/// This trait defines an interface for any algorithm query in the KD-Tree. In a
/// KD-Tree, the `query` function takes a visitor that implements this trait as
/// well as a reference point that is used to navigate the tree. In the visitor,
/// parameters this reference point is called `other`.
/// 
/// The `accept` function may modify the visitor in some way. The only way to
/// get nodes out of the tree is through the accept function. For instance, if a
/// visitor wants all of the visited points, it can simply store all of the
/// points passed to the `accept` function.
/// 
/// The radius function should return a radius around the reference point that 
/// still needs to be searched through. If one wanted to get all of the points in
/// the tree, it would suffice to always return `f32::INFINITY` from `radius`.
/// 
/// The lifetime `'a` is the lifetime of the KD-Tree.
pub trait Visitor<'a, P> {
    /// The final result of the visitor.
    type Result;

    /// Return the radius around `other` that still has to be searched through.
    /// This function is assumed to be decreasing. If for the same `other` it's
    /// value increases, it is a logic error.
    fn radius<Q, const N: usize>(&self, other: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>;

    /// Accept a point with data `point` and index into the KD-Tree `index`. `other` is
    /// the reference point that is used to navigate the KD-Tree.
    fn accept<Q, const N: usize>(&mut self, point: &'a P, index: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>;

    /// Consume the visitor into the final result.
    fn result(self) -> Self::Result;
}

/// A visitor that returns the index of the nearest point to the reference point
pub struct NearestIndexedVisitor<'a, P> {
    min: Option<(usize, &'a P)>,
}

impl<'a, P> NearestIndexedVisitor<'a, P> {
    pub fn new() -> Self {
        NearestIndexedVisitor { min: None }
    }
}

impl<'a, P> Visitor<'a, P> for NearestIndexedVisitor<'a, P>
{
    type Result = Option<(usize, &'a P)>;

    fn radius<Q, const N: usize>(&self, other: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        if let Some((_, min)) = self.min {
            na::distance(&min.borrow().point(), &other.point())
        } else {
            f32::INFINITY
        }
    }

    fn accept<Q, const N: usize>(&mut self, point: &'a P, index: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let dist = na::distance(&point.borrow().point(), &other.point());
        if self.radius(other) > dist {
            self.min.replace((index, point));
        }
    }

    fn result(self) -> Option<(usize, &'a P)> {
        self.min
    }
}

/// A visitor that returns the nearest point to the reference point
pub struct NearestVisitor<'a, P> {
    inner: NearestIndexedVisitor<'a, P>,
}

impl<'a, P> NearestVisitor<'a, P> {
    pub fn new() -> Self {
        NearestVisitor { inner: NearestIndexedVisitor::new() }
    }
}

impl<'a, P> Visitor<'a, P> for NearestVisitor<'a, P>
{
    type Result = Option<&'a P>;

    fn radius<Q, const N: usize>(&self, other: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.inner.radius(other)
    }

    fn accept<Q, const N: usize>(&mut self, point: &'a P, index: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.inner.accept(point, index, other)
    }

    fn result(self) -> Option<&'a P> {
        self.inner.result().map(|r| r.1)
    }
}

pub struct WithinRadiusIndexedVisitor<'a, P> {
    within_radius: Vec<(usize, &'a P)>,
    radius: f32,
}

impl<'a, P> WithinRadiusIndexedVisitor<'a, P> {
    pub fn new(radius: f32) -> Self {
        WithinRadiusIndexedVisitor {
            within_radius: Vec::new(),
            radius,
        }
    }
}

impl<'a, P> Visitor<'a, P> for WithinRadiusIndexedVisitor<'a, P>
{
    type Result = Vec<(usize, &'a P)>;

    fn radius<Q, const N: usize>(&self, _: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.radius
    }

    fn accept<Q, const N: usize>(&mut self, point: &'a P, index: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let dist = na::distance_squared(&point.borrow().point(), &other.point());
        if dist <= self.radius.powi(2) {
            self.within_radius.push((index, point));
        }
    }

    fn result(self) -> Vec<(usize, &'a P)> {
        self.within_radius
    }
}

pub struct WithinRadiusVisitor<'a, P> {
    within_radius: Vec<&'a P>,
    radius: f32,
}

impl<'a, P> WithinRadiusVisitor<'a, P> {
    pub fn new(radius: f32) -> Self {
        WithinRadiusVisitor {
            within_radius: Vec::new(),
            radius,
        }
    }
}

impl<'a, P> Visitor<'a, P> for WithinRadiusVisitor<'a, P>
{
    type Result = Vec<&'a P>;

    fn radius<Q, const N: usize>(&self, _: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.radius
    }

    fn accept<Q, const N: usize>(&mut self, point: &'a P, _: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let dist = na::distance_squared(&point.borrow().point(), &other.point());
        if dist <= self.radius.powi(2) {
            self.within_radius.push(point);
        }
    }

    fn result(self) -> Vec<&'a P> {
        self.within_radius
    }
}

pub struct WithinRadiusIndicesVisitor {
    within_radius: Vec<usize>,
    radius: f32,
}

impl WithinRadiusIndicesVisitor {
    pub fn new(radius: f32) -> Self {
        WithinRadiusIndicesVisitor {
            within_radius: Vec::new(),
            radius,
        }
    }
}

impl<'a, P> Visitor<'a, P> for WithinRadiusIndicesVisitor
{
    type Result = Vec<usize>;

    fn radius<Q, const N: usize>(&self, _: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.radius
    }

    fn accept<Q, const N: usize>(&mut self, point: &'a P, index: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        let dist = na::distance_squared(&point.borrow().point(), &other.point());
        if dist <= self.radius.powi(2) {
            self.within_radius.push(index);
        }
    }

    fn result(self) -> Vec<usize> {
        self.within_radius
    }
}

pub struct PreconditionVisitor<V, F, P> {
    prec: F,
    vis: V,
    _marker: PhantomData<P>,
}

impl<'a, P, V, F> PreconditionVisitor<V, F, P>
where
    V: Visitor<'a, P>,
    for<'b> F: Fn(&'b P) -> bool,
{
    pub fn new(vis: V, prec: F) -> Self {
        PreconditionVisitor { prec, vis, _marker: PhantomData }
    }
}

impl<'a, P, V, F> Visitor<'a, P> for PreconditionVisitor<V, F, P>
where
    V: Visitor<'a, P>,
    for<'b> F: Fn(&'b P) -> bool,
{
    type Result = V::Result;

    fn radius<Q, const N: usize>(&self, other: &Q) -> f32
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        self.vis.radius(other)
    }

    fn accept<Q, const N: usize>(&mut self, point: &'a P, index: usize, other: &Q)
    where
        Q: HasCoords<N> + ?Sized,
        P: Borrow<Q>,
    {
        if (self.prec)(point) {
            self.vis.accept(point, index, other);
        }
    }

    fn result(self) -> V::Result {
        self.vis.result()
    }
}