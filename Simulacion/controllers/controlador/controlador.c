#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define TIME_STEP 64
#define GRID_SIZE 8  
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define SPEED 4.0
#define MAX_TURNS 100

typedef struct {
  int x, y;
} Point;

typedef struct {
  int x, y;
  int g, h, f;
  int parent_index;
} Node;

int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES];
  int open_count = 0;
  Node closed_list[GRID_SIZE * GRID_SIZE];
  int closed_count = 0;

  Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
  start_node.f = start_node.g + start_node.h;
  open_list[open_count++] = start_node;

  while (open_count > 0) {
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f)
        best_index = i;
    }

    Node current = open_list[best_index];
    for (int i = best_index; i < open_count - 1; i++)
      open_list[i] = open_list[i + 1];
    open_count--;
    closed_list[closed_count++] = current;

    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      Node n = current;
      while (n.parent_index != -1 && length < max_path_len) {
        path[length++] = (Point){n.x, n.y};
        n = closed_list[n.parent_index];
      }
      path[length++] = (Point){start.x, start.y};
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }

    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
        continue;
      if (grid[nx][ny] == 1)
        continue;

      int in_closed = 0;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = 1;
          break;
        }
      }
      if (in_closed) continue;

      int g = current.g + 1;
      int h = heuristic(nx, ny, goal.x, goal.y);
      int f = g + h;

      int in_open = -1;
      for (int i = 0; i < open_count; i++) {
        if (open_list[i].x == nx && open_list[i].y == ny) {
          in_open = i;
          break;
        }
      }

      if (in_open != -1) {
        if (f < open_list[in_open].f) {
          open_list[in_open].g = g;
          open_list[in_open].h = h;
          open_list[in_open].f = f;
          open_list[in_open].parent_index = closed_count - 1;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor = {nx, ny, g, h, f, closed_count - 1};
        open_list[open_count++] = neighbor;
      }
    }
  }
  return 0;
}

int main() {
  wb_robot_init();
  int i;

  // Motores
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }

  // Sensores ultrasónicos
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

  // LIDAR
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // Variables
  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[100];
  int path_length = 0;

  double origin_x = 0.0, origin_y = 0.0;
  bool initialized = false;
  bool reached_goal = false;

  // Registro de giros
  char turn_log[MAX_TURNS][20];
  int turn_count = 0;
  double last_left_speed = 0.0, last_right_speed = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pose = wb_gps_get_values(gps);
    double robot_x = pose[0];
    double robot_y = pose[1];  // plano X-Y

    // Definir zona meta
    bool in_goal_area = (robot_x >= -1.66 && robot_x <= -1.58 &&
                         robot_y >= -1.66 && robot_y <= -1.58);

    if (!initialized) {
      origin_x = robot_x;
      origin_y = robot_y;
      initialized = true;
    }

    // Mostrar posición solo si no está en la meta
    if (!in_goal_area)
      printf("Posición GPS - X: %.2f, Y: %.2f\n", robot_x, robot_y);

    // Si llegó a la meta
    if (in_goal_area && !reached_goal) {
      printf("\n¡Llegó a la meta!\nSecuencia de giros:\n");
      for (i = 0; i < turn_count; i++)
        printf("%s\n", turn_log[i]);
      reached_goal = true;

      // Detener robot
      for (i = 0; i < 4; i++)
        wb_motor_set_velocity(wheels[i], 0.0);
      break;  // salir del bucle principal
    }

    // Evaluar sensores
    bool ds_detect_near = false, lidar_detect_near = false;
    double ds_values[2];
    for (i = 0; i < 2; i++) {
      ds_values[i] = wb_distance_sensor_get_value(ds[i]);
    }
    if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
      ds_detect_near = true;

    // LIDAR
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);

    for (int i = 0; i < resolution; i++) {
      double angle = -fov / 2 + i * (fov / resolution);
      double dist = ranges[i];
      if (isinf(dist)) continue;

      if (dist < 1.0) {
        double obs_x = robot_x + dist * cos(angle);
        double obs_y = robot_y + dist * sin(angle);

        int cell_x = (int)((obs_x - origin_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        int cell_y = (int)((obs_y - origin_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);

        if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE)
          grid[cell_x][cell_y] = 1;
      }

      if (dist < 0.3)
        lidar_detect_near = true;
    }

    // Planificación
    int robot_cell_x = (int)((robot_x - origin_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
    int robot_cell_y = (int)((robot_y - origin_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
    Point start = {robot_cell_x, robot_cell_y};
    Point goal = {GRID_SIZE - 2, GRID_SIZE - 2};
    path_length = plan_path(grid, start, goal, path, 100);

    // Movimiento
    double left_speed = SPEED;
    double right_speed = SPEED;

    if (lidar_detect_near || ds_detect_near) {
      left_speed = 1.0;
      right_speed = -1.0;
    }

    // Detectar giros y registrar
    if (turn_count < MAX_TURNS) {
      if (left_speed == 1.0 && right_speed == -1.0 && !(last_left_speed == 1.0 && last_right_speed == -1.0)) {
        strcpy(turn_log[turn_count++], "izquierda");
      } else if (left_speed == -1.0 && right_speed == 1.0 && !(last_left_speed == -1.0 && last_right_speed == 1.0)) {
        strcpy(turn_log[turn_count++], "derecha");
      }
    }

    last_left_speed = left_speed;
    last_right_speed = right_speed;

    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);

    fflush(stdout);
  }

  wb_robot_cleanup();
  return 0;
}
