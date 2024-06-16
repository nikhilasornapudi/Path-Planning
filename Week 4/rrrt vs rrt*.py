import math
import pygame
from pygame.math import Vector2
from random import randint
import time

WIDTH, HEIGHT = 800, 600  # Window size
OBSTACLE_COLOR = (255, 0, 0)  # Color of the obstacle
GOAL_RADIUS = 10  # Radius of goal position
pygame.display.set_caption("RRT vs RRT* Algorithm")


class Node:
    def __init__(self, position: Vector2, parent=None):
        self.position = position
        self.parent = parent

        if parent is not None:
            self.cost = parent.cost + (self.position - parent.position).length()
        else:
            self.cost = 0

    def path(self):
        node, p = self, []
        while node is not None:
            p.append(node)
            node = node.parent
        return p


def get_random_node():
    return Vector2(randint(0, WIDTH), randint(0, HEIGHT))


def get_nearest_node(nodes, random_node):
    return min(nodes, key=lambda node: (node.position - random_node).length())


def collision(src, dst, obstacles):
    """
    Check if the segment src - dst collides with any obstacle in obstacles.
    """
    points = [(i / 100) * dst + (1 - i / 100) * src for i in range(101)]
    w, h = obstacles.get_size()

    return any(0 <= p.x < w and 0 <= p.y < h and obstacles.get_at((int(p.x), int(p.y))) == OBSTACLE_COLOR for p in
               points)


def rrt(screen, start, goal, obstacles, max_iterations=5000):
    nodes = [Node(start)]
    start_time = time.time()

    for _ in range(max_iterations):
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        random_node = get_random_node()

        nearest_node = get_nearest_node(nodes, random_node)

        direction = (random_node - nearest_node.position).normalize()

        new_node = Node(nearest_node.position + direction * 20, nearest_node)

        if collision(new_node.position, nearest_node.position, obstacles):
            continue

        nodes.append(new_node)

        pygame.draw.line(screen, (255, 255, 255), new_node.parent.position, new_node.position)

        if (new_node.position - goal).length() <= GOAL_RADIUS:
            end_time = time.time()
            path = new_node.path()
            path_length = new_node.cost
            return path, path_length, end_time - start_time

    return None, None, None


def rrt_star(screen, start, goal, obstacles, max_iterations=5000, step_size=20):
    nodes = [Node(start)]
    start_time = time.time()

    for _ in range(max_iterations):
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        random_node = get_random_node()

        nearest_node = get_nearest_node(nodes, random_node)

        direction = (random_node - nearest_node.position).normalize()

        new_node = Node(nearest_node.position + direction * step_size, nearest_node)

        if collision(new_node.position, nearest_node.position, obstacles):
            continue

        nodes_in_range = [node for node in nodes if (node.position - new_node.position).length() <= step_size]

        if nodes_in_range:
            cost, node = min(
                ((node.cost + (node.position - new_node.position).length(), node) for node in nodes_in_range),
                key=lambda x: x[0]  # compare by the first element of the tuple
            )

            if not collision(node.position, new_node.position, obstacles):
                new_node.parent = node
                new_node.cost = cost

        nodes.append(new_node)

        pygame.draw.line(screen, (255, 255, 255), new_node.parent.position, new_node.position)

        if (new_node.position - goal).length() <= GOAL_RADIUS:
            end_time = time.time()
            path = new_node.path()
            path_length = new_node.cost
            return path, path_length, end_time - start_time

    return None, None, None


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))

    # Create obstacles surface (for now empty)
    obstacles = pygame.Surface((WIDTH, HEIGHT))

    start = None
    goal = None
    running = True
    path_rrt, path_length_rrt, time_rrt = None, None, None
    path_rrt_star, path_length_rrt_star, time_rrt_star = None, None, None

    font = pygame.font.Font(None, 24)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                if start is None:
                    start = Vector2(pos)
                    pygame.draw.circle(screen, (0, 255, 0), start, GOAL_RADIUS)
                elif goal is None:
                    goal = Vector2(pos)
                    pygame.draw.circle(screen, (0, 0, 255), goal, GOAL_RADIUS)
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE and start is not None and goal is not None:
                    if path_rrt is None:
                        path_rrt, path_length_rrt, time_rrt = rrt(screen, start, goal, obstacles)
                        if path_rrt is not None:
                            print("RRT Algorithm:")
                            print("Path Length:", path_length_rrt)
                            print("Time Taken:", time_rrt, "seconds")
                            for i in range(len(path_rrt) - 1):
                                pygame.draw.line(screen, (255, 0, 0), path_rrt[i].position, path_rrt[i + 1].position, width=5)
                                pygame.display.flip()
                        else:
                            print("No path found for RRT Algorithm.")

                    elif path_rrt_star is None:
                        path_rrt_star, path_length_rrt_star, time_rrt_star = rrt_star(screen, start, goal, obstacles)
                        if path_rrt_star is not None:
                            print("RRT* Algorithm:")
                            print("Path Length:", path_length_rrt_star)
                            print("Time Taken:", time_rrt_star, "seconds")
                            for i in range(len(path_rrt_star) - 1):
                                pygame.draw.line(screen, (0, 255, 255), path_rrt_star[i].position,
                                                 path_rrt_star[i + 1].position, width=5)
                                pygame.display.flip()
                        else:
                            print("No path found for RRT* Algorithm.")

        if path_length_rrt is not None:
            text_rrt = font.render(f"RRT: Path Length = {path_length_rrt:.2f}", True, (255, 255, 0))
            screen.blit(text_rrt, (10, 10))
        if time_rrt is not None:
            text_rrt_time = font.render(f"Time Taken = {time_rrt:.2f} seconds", True, (255, 255, 0))
            screen.blit(text_rrt_time, (10, 40))

        if path_length_rrt_star is not None:
            text_rrt_star = font.render(f"RRT*: Path Length = {path_length_rrt_star:.2f}", True, (255, 255, 0))
            screen.blit(text_rrt_star, (10, 130))
        if time_rrt_star is not None:
            text_rrt_star_time = font.render(f"Time Taken = {time_rrt_star:.2f} seconds", True, (255, 255, 0))
            screen.blit(text_rrt_star_time, (10, 160))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
