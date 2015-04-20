
import math

try:
    import Queue as Q  # ver. < 3.0
except ImportError:
    import queue as Q

def euclidian (p1, p2):
  x1, y1 = p1
  x_dest, y_dest = p2
  return math.sqrt ( (x1- x_dest)**2 + (y1- y_dest)**2 ) 

def find_path(src, dest, mesh):
  """
    Return the optimal path using bidirectional A* searching, sounds pretty awesome
  """

  def get_cost(box, goal):
    """
      Return cost of actual distance and heuristic (estimation)
    """
    if goal is 'destination':
      distance = distance_from_src[box]
      heuristic = get_heuristic (detail_points[box], dest)
    else:
      distance = distance_from_dest[box]
      heuristic = get_heuristic (detail_points[box], src)

    return  distance + heuristic

  def get_dist_travelled (box, goal):
    """
      Return actual distance from a detail_point of current box to target( source or destination)
    """
    dist = 0

    if goal is 'destination':
      while forward_prev_box[box]:
        dist += euclidian (detail_points[box], detail_points[forward_prev_box[box]])
        box = forward_prev_box[box]

    else:
      while backward_prev_box[box]:
        dist += euclidian (detail_points[box], detail_points[backward_prev_box[box]])
        box = backward_prev_box[box]

    return dist

  def get_heuristic (node, dest):
    return euclidian (node, dest)

  # function begins  
  x_src, y_src = src
  x_dest, y_dest = dest

  src_box  = None
  dest_box = None

  for x1, x2, y1, y2 in mesh['boxes']:
    if x_src >= x1 and x_src < x2 and y_src >= y1 and y_src < y2:
      src_box = (x1, x2, y1, y2)
     
    if  x_dest >= x1 and x_dest < x2 and y_dest >= y1 and y_dest < y2:
      dest_box = (x1, x2, y1, y2)

    if src_box and dest_box:
      break

  if not src_box or not dest_box:
    print "Source or destination is not available."
    return ([],[])

  if src_box == dest_box:
    print "They are in the same box."
    return ( [(src, dest)], [(src_box)] )

  
  # keep track of the boxes that are expanded
  visited_boxes = []
  visited_boxes.append(src_box)

  # dictionaries that store value for A* algorithm 
  distance_from_src = {} # key: a tuple of state, value: a cost(number) from the state to source state
  distance_from_dest = {}
  distance_from_src[src_box] = 0
  distance_from_dest[dest_box] = 0

  # gstore the nearest point(x, y) of a box (x1, x2, y1, y2) from given point (a, b) 
  detail_points = {} # key: a box (x1, x2, y1, y2), value: a detail point (x, y)
  detail_points[src_box] = src
  detail_points[dest_box] = dest

  forward_prev_box = {} #  key: a tuple of state , value: a tuple of state that the key comes from, from src to dest
  backward_prev_box = {}

  forward_prev_box[src_box] = None
  backward_prev_box[dest_box] = None

  # using only one priorityQueue to run bidirectional A*, strategy is called front-to-back bidirectional A*
  priorityQueue = Q.PriorityQueue() # item: ( totoalCost, a box, direction/goal )
  priorityQueue.put ( (get_cost(src_box, 'destination'), src_box, 'destination') )
  priorityQueue.put ( (get_cost(dest_box, 'source'), dest_box, 'source') )

  foundPath = False
  connectionBox = None

  while priorityQueue :
    cost, box, goal = priorityQueue.get()
    #list of boxes from adj
    adj_boxes = mesh['adj'].get(box, [])

    thisPrev = {}

    if goal is 'destination':
      thisPrev = forward_prev_box
    else:
      thisPrev = backward_prev_box

    if goal is 'destination' and box in backward_prev_box:
      foundPath = True
      connectionBox = box
      break

    if goal is 'source' and box in forward_prev_box:
      foundPath = True
      connectionBox = box
      break

    for adj_box in adj_boxes:

      if adj_box not in thisPrev:

        # determine detail point for adj_box
        x_p, y_p = detail_points[box]
        x1, x2, y1, y2 = adj_box
        detail_points[adj_box] = ( min(max(x_p, x1), x2), min(max(y_p, y1), y2) )

        if goal is 'destination':
          forward_prev_box[adj_box] = box
          distance_from_src[adj_box] = get_dist_travelled(adj_box, goal)
        else:
          backward_prev_box[adj_box] = box
          distance_from_dest[adj_box] = get_dist_travelled(adj_box, goal)

        visited_boxes.append((adj_box))
        priorityQueue.put( (get_cost(adj_box, goal), adj_box, goal) )

  if not foundPath:
    print "No Path!"
    return ([],[])

  else:
    path = []

    # calculating the two nearests points of the connection box from their parents' detail point
    x_p_f, y_p_f = detail_points[forward_prev_box[connectionBox]]
    x_p_b, y_p_b = detail_points[backward_prev_box[connectionBox]]
    x1, x2, y1, y2 = connectionBox
    c_detail_forward = ( min(max(x_p_f, x1), x2), min(max(y_p_f, y1), y2) )
    c_detail_backward = ( min(max(x_p_b, x1), x2), min(max(y_p_b, y1), y2) )

    # making line segments 
    path.append((c_detail_forward, c_detail_backward))
    path.append((c_detail_forward, detail_points[forward_prev_box[connectionBox]]))
    path.append((c_detail_backward, detail_points[backward_prev_box[connectionBox]]))

    box = forward_prev_box[connectionBox]
    while forward_prev_box[box] :
      path.append( ( detail_points[box], detail_points[forward_prev_box[box]] )) 
      box = forward_prev_box[box]

    box = backward_prev_box[connectionBox]
    while backward_prev_box[box] :
      path.append( ( detail_points[box], detail_points[backward_prev_box[box]] )) 
      box = backward_prev_box[box]

    return ( path, visited_boxes )

