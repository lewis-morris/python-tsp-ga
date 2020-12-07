# python-tsp

## A version of the "Travelling Salesman Problem" calculated with a genetic algorithm.

- Genetic algorithms mimic the reproductive traits of living organisms which, pass down genetic traits from two parents to their offspring.


#### Install dependencies

First navigate to a directory where you want to download it to...

```
git clone https://github.com/lewis-morris/python-tsp
cd python-tsp
pip install -r requirements.txt

```

#### Run Code


Basic:

In download directly run 

```
python tsp.py
```

Advanced:

```
optional arguments:
  -h, --help            show this help message and exit
  -c CITIES, --cities CITIES
                        This is the amount of 'cities' to add to the map or
                        the hardness of the calculation
  -w WORKERS, --workers WORKERS
                        The amount of workers per family generation
  -f FAMILIES, --families FAMILIES
                        The amount of families per generation
  -r RANDOM, --random RANDOM
                        Draw the cities randomly on the map or equally spaced.
                        0= equally (draws a perfect polygon of n sides, which
                        is easy to determine when complete) 1= randomly
                        (points are randomly selected in the 'map' space)
  -d DRAW, --draw DRAW  Draw the output onscreen. 0= no 1= yes
```
