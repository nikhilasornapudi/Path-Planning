import math
import pygame
from pygame.math import Vector2
from random import randint

WIDTH, HEIGHT = 800, 600  # Window size
OBSTACLE_COLOR = (255, 0, 0)  # Color of the obstacle
GOAL_RADIUS = 10  # Radius of goal position
pygame.display.set_caption("RRT* Algorithm")


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
    points = [(i/100) * dst + (1 - i/100) * src for i in range(101)]
    w, h = obstacles.get_size()

    return any(0 <= p.x < w and 0 <= p.y < h and obstacles.get_at((int(p.x), int(p.y))) == OBSTACLE_COLOR for p in points)


def rrt_star(screen, start, goal, obstacles, max_iterations=5000):

    nodes = [Node(start)]

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

        nodes_in_range = [node for node in nodes if (node.position - new_node.position).length() <= 50]

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
            return new_node.path()

    return None


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))

    # Create obstacles surface (for now empty)
    obstacles = pygame.Surface((WIDTH, HEIGHT))

    start = None
    goal = None
    running = True
    path = None

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
                if event.key == pygame.K_SPACE and start is not None and goal is not None and path is None:
                    path = rrt_star(screen, start, goal, obstacles)
                    if path is not None:
                        for i in range(len(path) - 1):
                            pygame.draw.line(screen, (255, 0, 0), path[i].position, path[i+1].position, width=5)
                            pygame.display.flip()
                    else:
                        print("No path found.")

    pygame.quit()


if __name__ == "__main__":
    main()
