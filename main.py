import datetime
import os

import cv2
import math
import random
import copy
import numpy as np


class City:
    """ holds the city coords etc """

    def __init__(self, maps, iden, point):
        self.iden = iden
        if point:
            self.x = point[0]
            self.y = point[1]
        else:
            self.x = random.randint(0, maps.w)
            self.y = random.randint(0, maps.h)

    def get_position(self):
        return self.y, self.x


class Map:
    """
    hold the map details

    """

    def __init__(self, h, w, cities=None, points=None):
        self.h = h
        self.w = w
        # number of cities
        self.no_cities = cities if points is None else len(points)
        # cities list
        self.cities = []
        # create cities, randomly if no points are passed
        for city in range(self.no_cities):
            self.cities.append(City(self, city, points[city] if not points is None else None))

    def get_distance(self, pos, pos2):
        # get distance between two cities
        city1 = self.cities[pos]
        city2 = self.cities[pos2]

        dist = math.hypot(city2.x - city1.x, city2.y - city1.y)
        return dist

    def get_distance_from_list(self, positions, add=True):
        # get distance from list of index positions of the cities
        return sum(
            [self.get_distance(x, x + 1) if x != len(positions) - 1 else self.get_distance(x, 0) for x in positions])


class Worker:
    def __init__(self, mapp):

        # randomly init their first route
        cities = [i for i, x in enumerate(mapp.cities)]
        random.shuffle(cities)
        self.route = cities
        self.mapp = mapp

    def get_distance(self, add=True):

        # get total distance for the current route assigned to this worker
        dist = []

        for i, x in enumerate(self.route):
            if i == len(self.route) - 1:
                dist.append(self.mapp.get_distance(x, self.route[0]))
            else:
                dist.append(self.mapp.get_distance(x, self.route[i + 1]))

        if add:
            return sum(dist)
        else:
            return dist

    def get_genes(self, worker1):
        # mix genes with the input worker. Choosing a random subset to keep.
        geneA = int(random.random() * len(self.route))
        geneB = int(random.random() * len(self.route))
        start = min(geneA, geneB)
        end = max(geneA, geneB)

        childP1 = []
        childP2 = []

        for i in range(start, end):
            childP1.append(self.route[i])

        childP2 = [x for x in worker1.route if x not in childP1]

        return childP1 + childP2

    def mutate(self, daddy_route):

        self.route = copy.copy(daddy_route)
        if random.randint(1, 100) == 1:
            random.shuffle(self.route)
        else:

            for i, x in enumerate(self.route):
                if random.randint(0, 100) < 5:
                    current = self.route[i]
                    nextt = current
                    while nextt != current:
                        nextt = random.randint(0, len(self.route) - 1)

                    self.route[i] = self.route[nextt]
                    self.route[nextt] = current

        return self


class Family:
    def __init__(self, workers, maps):
        self.workers = [Worker(maps) for x in range(workers)]
        self.eliete = []
        self.best_score = 999999

    def sort_workers(self):
        self.workers = sorted(self.workers, key=lambda item: item.get_distance())
        self.eliete = self.workers[:int(len(self.workers) * 0.02)]
        self.best_score = self.eliete[0].get_distance()
        self.best_worker = self.eliete[0]
        self.breed()

    def breed(self, mummy=None):
        random.shuffle(self.eliete)
        if mummy is None:
            the_best, the_second_best = self.eliete[:2]
        else:
            the_best = self.best_worker
            the_second_best = mummy

        best_fice_perc = self.workers[0:int(len(self.workers) * 0.05)]
        [work.mutate(the_best.get_genes(the_second_best)) for work in self.workers if work not in best_fice_perc]


def get_cities(maps):
    board = np.ones((maps.h, maps.w, 3), dtype=np.uint8) * 255
    for city in maps.cities:
        board = cv2.circle(board, (city.y, city.x), radius=int(maps.w * 0.01), color=(125, 125, 125), thickness=-1)
    return board


def show_best(board, best, maps):
    num_cols = len(best.route)
    blue = np.linspace(100, 250, num_cols)
    green = np.linspace(50, 200, num_cols)
    red = np.linspace(200, 50, num_cols)

    for i, x in enumerate(best.route):
        point1 = maps.cities[x].get_position()
        if i == len(maps.cities) - 1:
            point2 = maps.cities[best.route[0]].get_position()
        else:
            point2 = maps.cities[best.route[i + 1]].get_position()
        board = cv2.line(board, point1, point2, color=(int(blue[i]), int(green[i]), int(red[i])), thickness=3)

    # board = cv2.resize(board, dsize=(800, 800))
    cv2.imshow("test", board)
    # cv2.destroyAllWindows()


def get_round_points(no):
    from shapely.geometry import Point, MultiPoint
    from shapely.affinity import rotate
    point = Point(400, 400)
    starting_point = Point((40, 440))
    points_list = []
    for x in np.linspace(0, 360, no):
        points_list.append(rotate(starting_point, x, origin=point))
    return [[int(x.x), int(x.y)] for x in points_list]


def strfdelta(tdelta, fmt):
    d = {"days": tdelta.days}
    d["hours"], rem = divmod(tdelta.seconds, 3600)
    d["minutes"], d["seconds"] = divmod(rem, 60)
    return fmt.format(**d)


def breed_families(families, best_only=False):
    for i, x in enumerate(families):
        if i >= 1:
            if best_only:
                families[i].breed(mummy=families[int(random.random() * len(families))].best_worker)
            else:
                families[i].breed(mummy=families[i - 1].best_worker)
    return families


def run(cities, families=5, workers=200, random_points=False, display=True):
    # zero best score
    best_dist = 99999999

    # ser run no
    run = 0

    # load cities - if not random then evenly spaces (this way is easier to see if complete)
    if random_points:
        maps = Map(800, 800, cities, None)
    else:
        maps = Map(800, 800, None, get_round_points(cities))

    # load families
    families = [Family(workers, maps) for x in range(families)]

    # set display text
    best_text = f"Best distance : {best_dist} on generation # {run}"
    run_text = ""

    # get base board
    if display:
        print("Display loaded - click output and press 'q' to quit.")
        board_base = get_cities(maps)
    else:
        print("press 'ctrl + c' to quit")
    print("----------------------------------------------")

    # get start time
    time = datetime.datetime.utcnow()
    time2 = datetime.datetime.utcnow()
    time_taken_text = "Total run time : " + strfdelta((time2 - time), "{hours:02d}:{minutes:02d}:{seconds:02d}")

    # set running
    while True:

        # load board for display
        if display:
            board = board_base.copy()

        # calc each family - mutate/ breed etc (breeding is incestual at this point)
        _ = [fam.sort_workers() for fam in families]

        # sort from best to worst
        families = sorted(families, key=lambda item: item.best_score)

        # get best score
        best_score = families[0].best_score

        # print best score if better than last best
        if best_score < best_dist:
            best_dist = best_score
            best_text = f"Best distance : {best_dist:.2f} on generation # {run}"

        # increment run no
        run += 1

        if run % 20 == 0:
            # breed between families every 20 runs and update.
            families = breed_families(families)

        # get run text
        run_text = f" Current generation # {run - 1}"

        # display if needed
        if display:
            show_best(board, families[0].best_worker, maps)
            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                break

        # get run time
        time2 = datetime.datetime.utcnow()
        time_taken_text = "  -  Total run time : " + strfdelta((time2 - time),
                                                               "{hours:02d}:{minutes:02d}:{seconds:02d}")

        # print
        print("\r                                                                        ", end="")
        print("\r" + best_text + " : " + run_text + time_taken_text, end="")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="""Python Travelling Sales Man Genetic Algorithm -- \n\n
                                                    
                                                    
                                                    Calculating the shortest distance between n points.                                                     
                                                 """)
    parser.add_argument("-c", "--cities", default=50, type=int,
                        help="This is the amount of 'cities' to add to the map or the hardness of the calculation")
    parser.add_argument("-w", "--workers", default=200, type=int, help="The amount of workers per family generation")
    parser.add_argument("-f", "--families", default=5, type=int, help="The amount of families per generation")
    parser.add_argument("-r", "--random", default=0, type=int,
                        help="Draw the cities randomly on the map or equally spaced. 0= equally (draws a perfect polygon of n sides, which is easy to determine when complete) 1= randomly (points are randomly selected in the 'map' space)")
    parser.add_argument("-d", "--draw", default=1, type=int, help="Draw the output onscreen. 0= no 1= yes")

    args = parser.parse_args()
    cities = args.c
    workers = args.w
    families = args.f
    random_points = True if args.r == 1 else False
    draw = True if args.d == 1 else False

    run(cities, families, workers, random_points, draw)
