import math

try:
    import Queue as Q  # ver. < 3.0
except ImportError:
    import queue as Q

def get_detail_points(prev_detail_pt, box):
  """
    Based on previous detail point, return the nearest point to next box
  """
  x_p, y_p = prev_detail_pt
  x1, x2, y1, y2 = box
  return ( min(max(x_p, x1), x2), min(max(y_p, y1), y2) )

def euclidian (p1, p2):
  x1, y1 = p1
  x_dest, y_dest = p2
  return math.sqrt ((x1- x_dest)**2 + (y1- y_dest)**2 ) 

def find_path(src, dest, mesh):
  """
    Use awesome bidirectinal A* search with navigation box to find path from two points  
  """

  def get_cost(box, goal):
    """
      return the A* cost (total distance and estimation) from goal point to current point
    """
    curr_box = box

    if goal == 'dest':
      distance = distance_from_src[curr_box]
      heuristic = get_heuristic (detail_points[curr_box], dest)
    else:
      distance = distance_from_dest[curr_box]
      heuristic = get_heuristic (detail_points[curr_box], src)

    return  distance + heuristic

  def get_dist_travelled (box, goal):
    """
      return the total distance from goal point to current point
    """
    dist = 0
    curr_box = box
  
    if goal == "dest":
    
      while forward_prev_box[curr_box]:
        prev_box = forward_prev_box[curr_box]
        dist += euclidian (detail_points[curr_box], detail_points[prev_box])
        curr_box = prev_box
       
    else:
      while backward_prev_box[curr_box]:
        prev_box = backward_prev_box [curr_box]
        dist += euclidian (detail_points[curr_box], detail_points[prev_box])
        curr_box = prev_box

    return dist

  def get_heuristic (node, dest):
    return euclidian (node, dest)
 
  """
    ********* Here is where the function starts **********
  """
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

  #early check point
  if not src_box or not dest_box:
    print "Source or destination is not available."
    return ([],[])

  elif src_box == dest_box:
    print "They are in the same box."
    return ([(src, dest)], [(src_box)])

  
  visited_boxes = []
  visited_boxes.append(src_box)
  visited_boxes.append(dest_box)

  # store the nearest point(x, y) of a box (x1, x2, y1, y2) from given point (a, b) 
  detail_points = {} # key: a box (x1, x2, y1, y2), value: a detail point (x, y)
  detail_points[src_box] = src
  detail_points[dest_box] = dest

  forward_prev_box = {} # key: a box (x1, x2, y1, y2), value: the parent box
  backward_prev_box = {}
  forward_prev_box[src_box] = None
  backward_prev_box[dest_box] = None

  distance_from_src = {} # key: a box (x1, x2, y1, y2), value: distant from target point
  distance_from_dest = {}
  distance_from_src[src_box] = 0
  distance_from_dest[dest_box] = 0

  priorityQueue = Q.PriorityQueue()
  priorityQueue.put ( ( get_cost(src_box, "dest"), src_box, "dest") )
  priorityQueue.put ( ( get_cost(dest_box, "src"), dest_box, "src") )

  found_path = False
  connection_box = None

  while priorityQueue :
    cost, box, goal = priorityQueue.get()
   
    if box == dest_box and goal == 'dest' :
      found_path = True
      connection_box = dest_box
      break

    if box == src_box and goal == 'src':
      found_path =True
      connection_box = src_box
      break

    if goal == 'dest' and box in backward_prev_box.keys():
      found_path = True
      connection_box = box
      break

    if goal == 'src' and box in forward_prev_box.keys():
      found_path = True
      connection_box = box
      break
    
    this_prev = {}

    if goal == 'dest':
      this_prev = forward_prev_box
    else:
      this_prev = backward_prev_box

    #list of boxes from adj
    adj_boxes = mesh['adj'].get(box, [])

    for adj_box in adj_boxes:

      if adj_box not in this_prev.keys():

        detail_points[adj_box] = get_detail_points (detail_points[box], adj_box)
      
        if goal == "dest":
          forward_prev_box[adj_box] = box
          distance_from_src[adj_box] = get_dist_travelled(adj_box, goal)
        else:
          backward_prev_box[adj_box] = box
          distance_from_dest[adj_box] = get_dist_travelled(adj_box, goal)
        
        visited_boxes.append((adj_box))
        priorityQueue.put( (get_cost(adj_box, goal), adj_box, goal) )

  if not found_path:
    print "No Path!"
    return ([],[])

  else: # build line segments
    #return ([],[]);
    path = []

    if connection_box == src_box: 
      curr_box = connection_box

      while backward_prev_box[curr_box]:
        prev_box = backward_prev_box[curr_box]
        path.append( ( detail_points[curr_box], detail_points[prev_box] )) 
        curr_box = prev_box

      path.append ((src, detail_points[connection_box]))

    elif connection_box == dest_box:
      curr_box = connection_box
      while forward_prev_box[curr_box]:
        prev_box = forward_prev_box[curr_box]
        path.append( ( detail_points[curr_box], detail_points[prev_box] )) 
        curr_box = prev_box

      path.append ((dest, detail_points[connection_box]))

    else : # they meet at somewhere in the middle
  
      box_from_src = forward_prev_box[connection_box]
      box_from_dest = backward_prev_box[connection_box]
      point_close_to_src = detail_points[ box_from_src ]
      point_close_to_dest = detail_points [ box_from_dest]

      # calculating the two nearests points of the connection box from their parents' detail point
      c_detail_forward = get_detail_points( point_close_to_src, connection_box)
      c_detail_backward = get_detail_points( point_close_to_dest, connection_box)

      # making line segments 
      path.append((c_detail_forward, c_detail_backward))
      path.append((c_detail_forward, detail_points[forward_prev_box[connection_box]]))
      path.append((c_detail_backward, detail_points[backward_prev_box[connection_box]]))

      box = forward_prev_box[connection_box]
      while forward_prev_box[box] :
        path.append( ( detail_points[box], detail_points[forward_prev_box[box]] )) 
        box = forward_prev_box[box]

      box = backward_prev_box[connection_box]
      while backward_prev_box[box] :
        path.append( ( detail_points[box], detail_points[backward_prev_box[box]] )) 
        box = backward_prev_box[box]

   
    return ( path, visited_boxes )
