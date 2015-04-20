import sys
import math

try:
    import Queue as Q  # ver. < 3.0
except ImportError:
    import queue as Q

def find_path(src, dest, mesh):

  visited_boxes = []
  detail_points = {} # [box]: (x,y)
  previous_box = {}
  distance_from_src = {}
  x_src, y_src = src
  x_dest, y_dest = dest

  src_box = None
  dest_box = None


  priorityQueue = Q.PriorityQueue()

  def get_cost(box):
    distance = distance_from_src[box]
    heuristic = get_heuristic (detail_points[box], dest)
    return  distance + heuristic

  def get_dist_travelled (box):
    dist = 0

    while  previous_box[box]:
      dist += euclidian (detail_points[box], detail_points[previous_box[box]])
      box = previous_box[box]

    return dist

  def get_heuristic (node, dest):
    return euclidian (node, dest)

  
  for x1, x2, y1, y2 in mesh['boxes']:
    if x_src >= x1 and x_src < x2 and y_src >= y1 and y_src < y2:
      src_box = (x1, x2, y1, y2)
      visited_boxes.append(src_box)
    if  x_dest >= x1 and x_dest < x2 and y_dest >= y1 and y_dest < y2:
      dest_box = (x1, x2, y1, y2)

    if src_box and dest_box:
      break
  
  if not src_box or not dest_box:
    print "No Path!"
    return ([],[]);

  detail_points[src_box] = src
  detail_points[dest_box] = dest
  previous_box[src_box] = None

  distance_from_src[src_box] = 0
  priorityQueue.put ( (distance_from_src[src_box], src_box) )

  while  priorityQueue :
    cost, box =  priorityQueue.get()
    #list of boxes from adj
    adj_boxes = mesh['adj'].get(box, [])

    if box == dest_box:
      #return ([(src, dest)],visited_boxes)
      break

    for adj_box in adj_boxes:

      if adj_box not in visited_boxes:

        # determine detail point for adj_box
        x_p, y_p = detail_points[box]
        x1, x2, y1, y2 = adj_box
        detail_points[adj_box] = ( min(max(x_p, x1), x2), min(max(y_p, y1), y2) )
      
        previous_box[adj_box] = box
        distance_from_src[adj_box] = get_dist_travelled(adj_box)

        
        visited_boxes.append((adj_box))
        priorityQueue.put( (get_cost(adj_box), adj_box) )


  if box != dest_box:
    print "No Path!"
    return ([],[])

  else:
    #print detail_points.values()
    path = []
    box = dest_box
    path.append (( dest, detail_points[box]))

    while previous_box[box] :
      path.append( ( detail_points[box], detail_points[previous_box[box]] )) 
      box = previous_box[box]

    return ( path, visited_boxes)

 
def euclidian (p1, p2):
  x1, y1,  = p1
  x_dest, y_dest = p2
  return math.sqrt ( (x1- x_dest)**2 +  (y1- y_dest)**2 ) 





