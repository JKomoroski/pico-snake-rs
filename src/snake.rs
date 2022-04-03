use crate::direction::Direction;
use crate::point::Point;
use heapless::Deque;

#[derive(Debug)]
pub struct Snake {
    pub body: Deque<Point, 256>,
    direction: Direction,
    digesting: bool,
}

impl Snake {
    pub fn new(start: Point, length: u16, direction: Direction) -> Self {
        let opposite = direction.opposite();

        let mut body: Deque<Point, 256> = Deque::new();
        
        (0..length)
            .into_iter()
            .map(|i| start.transform(opposite, i))
            .for_each(|p| body.push_back(p).unwrap());

        Self { body, direction, digesting: false }
    }

    pub fn get_head_point(&self) -> Point {
        self.body.front().unwrap().clone()
    }

    pub fn get_body_points(&self) -> Deque<Point, 256> {
        self.body.clone()
    }

    pub fn get_direction(&self) -> Direction {
        self.direction.clone()
    }

    pub fn contains_point(&self, point: &Point) -> bool {
        self.body.iter().any(|p| p == point)
    }

    pub fn slither(&mut self) {
        self.body.push_front(self.body.front().unwrap().transform(self.direction, 1)).unwrap();
        if !self.digesting {
            self.body.pop_back();
        } else {
            self.digesting = false;
        }
    }

    pub fn set_direction(&mut self, direction: Direction) {
        self.direction = direction;
    }

    pub fn grow(&mut self) {
        self.digesting = true;
    }
}

/*

    fn draw_snake(&mut self) {

        let body_points = self.snake.get_body_points();
        for (i, body) in body_points.iter().enumerate() {
            let previous = if i == 0 { None } else { body_points.get(i - 1) };
            let next = body_points.get(i + 1);
            let symbol = if let Some(&next) = next {
                if let Some(&previous) = previous {
                    if previous.x == next.x {
                        '║'
                    } else if previous.y == next.y {
                        '═'
                    } else {
                        let d = body.transform(Direction::Down, 1);
                        let r = body.transform(Direction::Right, 1);
                        let u = if body.y == 0 {
                            body.clone()
                        } else {
                            body.transform(Direction::Up, 1)
                        };
                        let l = if body.x == 0 {
                            body.clone()
                        } else {
                            body.transform(Direction::Left, 1)
                        };
                        if (next == d && previous == r) || (previous == d && next == r) {
                            '╔'
                        } else if (next == d && previous == l) || (previous == d && next == l) {
                            '╗'
                        } else if (next == u && previous == r) || (previous == u && next == r) {
                            '╚'
                        } else {
                            '╝'
                        }
                    }
                } else {
                    'O'
                }
            } else if let Some(&previous) = previous {
                if body.y == previous.y {
                    '═'
                } else {
                    '║'
                }
            } else {
                panic!("Invalid snake body point.");
            };

        }
    }

*/