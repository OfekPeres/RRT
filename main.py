import numpy as np
from src.RRT import RRT

payload = {}

payload['obstacles'] = [{"shape":"rectangle", "definition":[20,10,40,20]},{"shape":"circle", "definition":[10,10,3]}, {"shape":"circle", "definition":[50,50,20]}]
payload['start'] = [0,0]
payload['goal'] = [90,25]
payload['goalRadius'] = 2
payload['d_max'] = 1
payload['width'] = 400
payload['height'] = 400



if __name__ == "__main__":
    x_0 = np.array([0,0])
    goal = np.array([90,25])
    goal_radius = 2 
    d_max = 1
    rrt = RRT(payload['start'], payload['goal'], payload['goalRadius'], payload['obstacles'], payload['d_max'], payload['width'],payload['height'])
    rrt.PlotRRT()
    