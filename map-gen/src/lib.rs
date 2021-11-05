use nalgebra as na;

pub mod sgl_parser;

pub use sgl_parser::parse_sgl;

#[cfg(test)]
mod tests {
    use super::*;

    use rrt::obstacle::Polygon;

    #[test]
    fn test_parsing() {
        let input = r"<number of polygons>
4
<x..., y..., n = 4, id = 0>
-4.35,-4.94,-3.16,-2.57
-4.28,-2.5,-1.91,-3.69
<x..., y..., n = 4, id = 1>
1.13,-0.31,-1.09,0.34
-3.52,-4.31,-2.86,-2.07
<x..., y..., n = 4, id = 2>
-1.33,-2.11,-1.01,-0.23
-6.15,-5.06,-4.28,-5.37
<x..., y..., n = 4, id = 3>
3.37,2.23,3.51,4.66
-9.16,-7.87,-6.73,-8.02";

        let result = parse_sgl::<2>(input);
        assert!(result.is_some());

        let polygons = result.unwrap();
        assert_eq!(
            polygons[&0],
            Polygon {
                vertices: vec![
                    na::Point2::new(-4.35, -4.28),
                    na::Point2::new(-4.94, -2.5),
                    na::Point2::new(-3.16, -1.91),
                    na::Point2::new(-2.57, -3.69),
                ]
            }
        )
    }
}
