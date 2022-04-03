use crate::direction::Direction;
use crate::point::Point;
use crate::snake::Snake;
use rand::{Rng, RngCore, SeedableRng};
use wyhash::WyRng;

const MAX_SPEED: u16 = 20;

pub struct Game {
    width: u16,
    height: u16,
    pub food: Option<Point>,
    pub snake: Snake,
    speed: u16,
    pub score: u16,
    pub done: bool,
    rng: WyRng,
}

impl Game {
    pub fn new(width: u16, height: u16, seed: u64) -> Self {
        let mut rng = WyRng::seed_from_u64(seed);
        let mut s = Self {
            width,
            height,
            food: None,
            snake: Snake::new(
                Point::new(width / 2, height / 2),
                3,
                match rng.next_u32() % 4 {
                    0 => Direction::Up,
                    1 => Direction::Right,
                    2 => Direction::Down,
                    _ => Direction::Left,
                },
            ),
            speed: 0,
            score: 0,
            done: false,
            rng: rng,
        };
        s.place_food();
        s
    }

    pub fn tick(&mut self, command: Command) {
        let direction = self.snake.get_direction();

        match command {
            Command::Quit => {
                self.done = true;
            }

            Command::Turn(towards) => {
                if direction != towards && direction.opposite() != towards {
                    self.snake.set_direction(towards);
                }
            }
        }

        if self.has_collided_with_wall() || self.has_bitten_itself() {
            self.done = true;
        } else {
            self.snake.slither();

            if let Some(food_point) = self.food {
                if self.snake.get_head_point() == food_point {
                    self.snake.grow();
                    self.place_food();
                    self.score += 1;

                    if self.score % ((self.width * self.height) / MAX_SPEED) == 0 {
                        self.speed += 1;
                    }
                }
            }
        }
    }

    fn has_collided_with_wall(&self) -> bool {
        let head_point = self.snake.get_head_point();

        match self.snake.get_direction() {
            Direction::Up => head_point.y == 0,
            Direction::Right => head_point.x == self.width - 1,
            Direction::Down => head_point.y == self.height - 1,
            Direction::Left => head_point.x == 0,
        }
    }

    fn has_bitten_itself(&self) -> bool {
        let next_head_point = self
            .snake
            .get_head_point()
            .transform(self.snake.get_direction(), 1);
        let mut next_body_points = self.snake.get_body_points().clone();
        next_body_points.pop_back();
        next_body_points.pop_front();

        next_body_points.iter().any(|p| p == &next_head_point)
    }

    fn place_food(&mut self) {
        loop {
            let random_x = self.rng.gen_range(0..self.width);
            let random_y = self.rng.gen_range(0..self.height);
            let point = Point::new(random_x, random_y);
            if !self.snake.contains_point(&point) {
                self.food = Some(point);
                break;
            }
        }
    }
}

pub enum Command {
    Quit,
    Turn(Direction),
}