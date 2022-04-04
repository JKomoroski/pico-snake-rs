use core::ops::Mul;

use crate::direction::Direction;

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
pub struct Point {
    pub x: u16,
    pub y: u16,
}

impl Point {
    pub fn new(x: u16, y: u16) -> Self {
        Self { x, y }
    }

    pub fn transform(&self, direction: Direction, times: u16) -> Self {
        let times = times as i16;
        let transformation = match direction {
            Direction::Up => (0, -times),
            Direction::Right => (times, 0),
            Direction::Down => (0, times),
            Direction::Left => (-times, 0),
        };

        Self::new(
            Self::transform_value(self.x, transformation.0),
            Self::transform_value(self.y, transformation.1),
        )
    }

    fn transform_value(value: u16, by: i16) -> u16 {
        if by.is_negative() && by.abs() as u16 > value {
            panic!("Transforming value {} by {} would result in a negative number", value, by);
        } else {
            (value as i16 + by) as u16
        }
    }

}

impl From<Point> for embedded_graphics::geometry::Point {
    fn from(p: Point) -> Self {
        embedded_graphics::geometry::Point::new(p.x.into(), p.y.into())
    }
}

impl Mul<embedded_graphics::geometry::Point> for Point {
    type Output = embedded_graphics::geometry::Point;

    fn mul(self, p: embedded_graphics::geometry::Point) -> embedded_graphics::geometry::Point {
        let x:i32 = p.x * self.x as i32;
        let y:i32 = p.y * self.y as i32;
        embedded_graphics::geometry::Point::new(x, y)
    }

}

impl Mul<Point> for Point {
    type Output = Point;

    fn mul(self, p: Point) -> Point {
        Point::new(self.x * p.x, self.y * p.y)
    }

}