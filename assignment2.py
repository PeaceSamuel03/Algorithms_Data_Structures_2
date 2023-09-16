#student name: Peace Samuel
#student number: 121376141
"""TASK ONE:Evaluating structures for Dijkstra's algorithm on simple weighted graphs"""
from stack import *
import sys, time

#Vertex class from lab solution
class Vertex:
    """ A Vertex in a graph. """
    
    def __init__(self, element):
        """ Create a vertex, with data element. """
        self._element = element

    def __str__(self):
        """ Return a string representation of the vertex. """
        return str(self._element)

    def __lt__(self, v):
        """ Return true if this object is less than v.
       
        Args:
            v -- a vertex object
        """
        return self._element < v.element()

    def element(self):
        """ Return the data for the vertex. """
        return self._element
    
#Edge class from lab solution
class Edge:
    """ An edge in a graph.

    Implemented with an order, so can be used for directed or undirected
    graphs. Methods are provided for both. It is the job of the Graph class
    to handle them as directed or undirected.
    """
    
    def __init__(self, v, w, element):
        """ Create an edge between vertices v and w, with label element.

        Args:
            element -- the label, can be an arbitrarily complex structure.
        """
        self._vertices = (v,w)
        self._element = element

    def __str__(self):
        """ Return a string representation of this edge. """
        return ('(' + str(self._vertices[0]) + '--'
                   + str(self._vertices[1]) + ' : '
                   + str(self._element) + ')')

    def vertices(self):
        """ Return an ordered pair of the vertices of this edge. """
        return self._vertices

    def start(self):
        """ Return the first vertex in the ordered pair. """
        return self._vertices[0]

    def end(self):
        """ Return the second vertex in the ordered. pair. """
        return self._vertices[1]

    def opposite(self, v):
        """ Return the opposite vertex to v in this edge. """
        if self._vertices[0] == v:
            return self._vertices[1]
        elif self._vertices[1] == v:
            return self._vertices[0]
        else:
            return None

    def element(self):
        """ Return the data element for this edge. """
        return self._element
    
#Graph class from lab solution
class Graph:
    """ Represent a simple graph.

        This version maintains only undirected graphs, and assumes no
        self loops.
    """

    #Implement as a Python dictionary
    #  - the keys are the vertices
    #  - the values are the edge sets for that vertex
    #         Each edge set is also maintained as a dictionary,
    #         with opposite vertex as the key and the edge object as the value
    
    def __init__(self):
        """ Create an initial empty graph. """
        self._structure = dict()

    def __str__(self):
        """ Return a string representation of the graph. """
        hstr = ('|V| = ' + str(self.num_vertices())
                + '; |E| = ' + str(self.num_edges()))
        vstr = '\nVertices: '
        for v in self._structure:
            vstr += str(v) + '-'
        edges = self.edges()
        estr = '\nEdges: '
        for e in edges:
            estr += str(e) + ' '
        return hstr + vstr + estr

    #--------------------------------------------------#
    #ADT methods to query the graph
    
    def num_vertices(self):
        """ Return the number of vertices in the graph. """
        return len(self._structure)

    def num_edges(self):
        """ Return the number of edges in the graph. """
        num = 0
        for v in self._structure:
            num += len(self._structure[v])    #the dict of edges for v
        return num //2     #divide by 2, since each edge appears in the
                           #vertex list for both of its vertices

    def vertices(self):
        """ Return a list of all vertices in the graph. """
        return [key for key in self._structure]
#########################################################################
    def get_vertex_by_label(self, element):
        """ get the first vertex that matches element. 
        
        Beware - this method is inefficient, and will be really noticeable
        if used on large graphs.
        """
        for v in self._structure:
            if v.element() == element:
                return v
        return None
########################################################################
    def edges(self):
        """ Return a list of all edges in the graph. """
        edgelist = []
        for v in self._structure:
            for w in self._structure[v]:
                #to avoid duplicates, only return if v is the first vertex
                if self._structure[v][w].start() == v:
                    edgelist.append(self._structure[v][w])
        return edgelist

    def get_edges(self, v):
        """ Return a list of all edges incident on v.

        Args:
            v -- a vertex object
        """
        if v in self._structure.keys():
            edgelist = []
            for w in self._structure[v]:
                edgelist.append(self._structure[v][w])
            return edgelist
        return None

    def get_edge(self, v, w):
        """ Return the edge between v and w, or None, if there is no edge.

        Args:
            v -- a Vertex object
            w -- a Vertex object
        """
        if (self._structure != None
                         and v in self._structure
                         and w in self._structure[v]):
            return self._structure[v][w]
        return None

    def degree(self, v):
        """ Return the degree of vertex v. """
        return len(self._structure[v])

    #--------------------------------------------------#
    #ADT methods to modify the graph
    
    def add_vertex(self, element):
        """ Add and return a new vertex with data element.

        Note -- if there is already a vertex with the same data element,
        this will create another vertex instance with the same element.
        If the client using this ADT implementation does not want these 
        duplicates, it is the client's responsibility not to add duplicates.
        """
        v = Vertex(element)
        self._structure[v] = dict()  # create an empty dict, ready for edges
        return v

    def add_vertex_if_new(self, element):
        """ Add and return a vertex with element, if not already in graph.

        Checks for equality between the elements. If there is special
        meaning to parts of the element (e.g. element is a tuple, with an
        'id' in cell 0), then this method may create multiple vertices with
        the same 'id' if any other parts of element are different.

        To ensure vertices are unique for individual parts of element,
        separate methods need to be written.

        Beware -- this will be inefficient for large graphs.
        """
        for v in self._structure:
            if v.element() == element:
                #print('Already there')
                return v
        return self.add_vertex(element)

    def add_edge(self, v, w, element):
        """ Add and return an edge, with element, between two vertices v and w.

        If either v or w are not vertices in the graph, does not add, and
        returns None.
            
        If an edge already exists between v and w, this will
        replace the previous edge.
        """
        if not v in self._structure or not w in self._structure:
            return None
        e = Edge(v, w, element)
        # self._structure[v] is the dictionary of v's edges
        # so need to insert an entry for key w, with value e
        # A clearer way of expressing it would be
        # v_edges = self._structure[v]
        # v_edges[w] = e
        self._structure[v][w] = e  
        self._structure[w][v] = e
        return e

    def add_edge_pairs(self, elist):
        """ Add all vertex pairs in elist as edges with empty elements. """
        for (v,w) in elist:
            self.add_edge(v,w,None)

    #--------------------------------------------------#
    #Additional methods to explore the graph
        
    def highestdegreevertex(self):
        """ Return the vertex with highest degree. """
        hd = -1
        hdv = None
        for v in self._structure:
            if self.degree(v) > hd:
                hd = self.degree(v)
                hdv = v
        return hdv            

    # --------------------------------------------------#
    # Traversal methods
    
    def dfs_stack(self, v):
        """ Return a DFS tree from v, using a stack.        """
        marked = {}
        stack = Stack()
        stack.push((v,None))
        # print('   pushed', v, 'from None')
        while stack.length() > 0:
            (vertex,edge) = stack.pop()
            if vertex not in marked:
                # print('popped unvisited', vertex)
                marked[vertex] = edge
                for e in self.get_edges(vertex):
                    w = e.opposite(vertex)
                    stack.push((w,e))
                    # print('   pushed', w, 'from', e)
        return marked

    def depthfirstsearch(self, v):
        """ Return a DFS tree from v. """
        marked = {v:None}
        self._depthfirstsearch(v, marked)
        return marked

    def _depthfirstsearch(self, v, marked):
        """ Do a recursive DFS from v, storing nodes in marked. """
        for e in self.get_edges(v):
            w = e.opposite(v)
            if w not in marked:
                marked[w] = e
                self._depthfirstsearch(w, marked)
                
    def breadthfirstsearch(self, v):
        """ Return a BFS tree from v. """
        marked = {v:None}
        level = [v]
        while len(level) > 0:
            nextlevel = []
            for w in level:
                for e in self.get_edges(w):
                    x = e.opposite(w)
                    if x not in marked:
                        marked[x] = e
                        nextlevel.append(x)
            level = nextlevel
        return marked

    def BFS_length(self, v):
        """ Return a BFS tree from v, with path lengths. 
        
        In the returned dictionary, each vertex (key) stores its parent (value).
        """
        marked = {v:(None,0)}
        level = [v]
        levelint = 1
        while len(level) > 0:
            nextlevel = []
            for w in level:
                for e in self.get_edges(w):
                    x = e.opposite(w)
                    if x not in marked:
                        marked[x] = (w, levelint)
                        nextlevel.append(x)
            level = nextlevel
            levelint += 1
        return marked
    
    def breadthfirstsearchtree(self, v):
        """ Return a down-directed BFS tree from v. 
        
        In the returned dictionary, each vertex (key) stores a list of its edge 'children'.
        """
        marked = {v:[]}
        level = [v]
        while len(level) > 0:
            nextlevel = []
            for w in level:
                for e in self.get_edges(w):
                    x = e.opposite(w)
                    if x not in marked:
                        marked[x] = []
                        marked[w].append(x)
                        nextlevel.append(x)
            level = nextlevel
        return marked

#graphreader function from assignment file
def graphreader(filename):
    """ Read and return the route map in filename. """
    graph = Graph()
    file = open(filename, 'r')
    entry = file.readline() #either 'Node' or 'Edge'
    num = 0
    while entry == 'Node\n':
        num += 1
        nodeid = int(file.readline().split()[1])
        vertex = graph.add_vertex(nodeid)
        entry = file.readline() #either 'Node' or 'Edge'
    #print('Read', num, 'vertices and added into the graph')
    num = 0
    while entry == 'Edge\n':
        num += 1
        source = int(file.readline().split()[1])
        sv = graph.get_vertex_by_label(source)
        target = int(file.readline().split()[1])
        tv = graph.get_vertex_by_label(target)
        length = float(file.readline().split()[1])
        edge = graph.add_edge(sv, tv, length)
        file.readline() #read the one-way data
        entry = file.readline() #either 'Node' or 'Edge'
    #print('Read', num, 'edges and added into the graph')
    print(graph)
    return graph

#Element class from lecture notes
class Element:
    """A key(priority), value(represents item) and index(location information)"""
    def __init__(self, k, v, i):
        self._key = k
        self._value = v
        self._index = i
    def __str__(self):
        return self._value
    # def __eq__(self, other):
    #     return self._key == other._key
    def __lt__(self, other):
        return self._key < other._key
    def _wipe(self):
        self._key = None
        self._value = None
        self._index = None


"""Unsorted list implementation"""
class AdaptablePriorityQueue1:
    def __init__(self):
        self._body = []
        self._size = 0
        self._end = 0 # index of the end of the list, free cell

    def print_(self):
        for element in self._body:
            print("cost:",element._key,"vertex:",element._value)

    def length(self):
        """return the number of items in the APQ"""
        return self._size
    
    def add(self, key, item):
        """add a new item to the APQ"""
        if len(self._body) == 0:
            element = Element(key, item, 0)#start index is zero
            self._body.append(element)#add element into the list
            self._end = 1 #next free index
        else:
            element = Element(key, item, self._end) #index of element, matches self._end
            self._end += 1# free index moves over each time we add an new item, next available cell
            self._body.append(element)
            self._size += 1
        return element
    
    def get_key(self, element:Element):
        """return the current key for the element"""
        return element._key
    
    def min(self):
        """return the element with the minimum key"""
        #set the first element as the temporary min
        min1 = self._body[0]
        #check each element in the list
        for element in self._body:
            #if another element is smaller, it becomes the new min
            if element._key < min1._key:
                min1 = element
        return min1
    
    def remove_min(self):
        """remove element and return the (key, value) with the min key"""
        if self._body == 0:
            return "APQ is empty"
        else:
            min:Element = self.min() #method above to find value with min key
            self._body.pop(min._index) #removes item by index
            self._size -= 1
            return min._key, min._value
        
    def update_key(self, element:Element, newkey):
        """updates key in element to be newkey"""
        element._key = newkey

    def remove(self,element:Element):
        """removes the element from the APQ"""
        if self._body == 0:
            return "APQ is empty"
        else:
            self._body.pop(element._index) #removes element by index
            self._size -= 1

"""Heap based implementation"""
class AdaptablePriorityQueue2:
    def __init__(self):
        self.queue = []#body of the heap queue
        self._end = 0

    def length(self):
        return len(self.queue)

    def get_key(self, element:Element):
        """return the current key for the element"""
        return element._key

    def add(self, priority, item):
        """add a new item to the APQ"""
        element = Element(priority, item, self._end)#create element to add to APQ
        self.queue.append(element)
        self._end += 1 #next available index
        self._bubble_up(len(self.queue) - 1)

    def remove_min(self):
        """remove and return element with the min key"""
        if  len(self.queue) == 0:
            return "Queue is empty"
        #find priority and value of the min element in the queue, top of the heap
        priority, item = (self.queue[0])._key , (self.queue[0])._value
        if len(self.queue) > 1:
            #remove min value, reorganize the heap
            self.queue[0] = self.queue.pop()
            self._bubble_down(0)
        else:
            self.queue.pop()
        #return the min element
        return  item

    def change_priority(self, item, new_priority):
        """updates key to new_priority"""
        element:Element
        for i, element in enumerate(self.queue):
            #enumerate -> (index, queue[index])
            if element._value == item:#item mut equal element in the queue
                #overwrite with the new priority/key at index
                self.queue[i] = Element(new_priority, item, i)
                #reorganize the heap
                if new_priority < element._key:
                    self._bubble_up(i)
                else:
                    self._bubble_down(i)
                return
        return "item not in queue"

    def _bubble_up(self, i):
        """method used to reorganize the heap"""
        while i > 0:
            #find parent index
            parent = (i - 1) // 2
            if self.queue[i]._key < self.queue[parent]._key:
                #swap the element and the parent
                self.queue[i], self.queue[parent] = self.queue[parent], self.queue[i]
                i = parent
            else:
                break

    def _bubble_down(self, i):
        """method used to reorganize the heap"""
        while 2 * i + 1 < len(self.queue):
            #get left child's  and right child's indexes
            left_child = 2 * i + 1
            right_child = 2 * i + 2 if 2 * i + 2 < len(self.queue) else None
            swap = i
            if self.queue[left_child]._key < self.queue[i]._key:
                #determine whether the left child is smaller the the current one
                swap = left_child
            if right_child is not None and self.queue[right_child]._key < self.queue[swap]._key:
                #determine whether the right child is smaller than the current one
                swap = right_child
            if swap != i:
                #swap the elements
                self.queue[i], self.queue[swap] = self.queue[swap], self.queue[i]
                i = swap
            else:
                break

#based on pseudo code in class lecture !!!!!!!!!!!!!!!!!!!!!!!!!!!! has problems :(
def Dijkstra(source_v:Vertex, dest_v:Vertex, bool_arg:bool, graph:Graph):
    """Based on the pseudo-code given in Lecture 14""" 
    if bool_arg == False:
        #False -> use unsorted list APQ
        print(graph)
        open = AdaptablePriorityQueue1()
        closed = {} #keys:Vertex, values: (cost, predecessor)
        location = {}#keys: vertices, values: location in open
        preds = {source_v: None} #no predecessor for source vertex
        s = open.add(0, source_v) #add source_v with key 0 to APQ
        location[source_v] = s #add s to locs, element returned from APQ
        while open is not None: #while open is not empty
            for key, value in closed.items():
                print("key:", key._element,"value:(",value[0], ",", (value[1]), ")")
            #remove the min element v and its cost (key) from open
            cost, v = open.remove_min()
            #remove the entry for v from locs and preds (which returns predecessor)
            location.pop(v)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!KEYERROR?? must fix this
            predecessor = preds.pop(v)
            #add an entry for v:(cost, predecessor) into closed
            closed[v] = (cost, predecessor)
            for edge in graph.get_edges(v):#for each edge e from v 
                #w is the opposite vertex to v in e
                w = edge.opposite(v)
                if w not in closed: #if w is not in closed
                    #newcost id v's key(cost) plus e's cost
                    newcost = cost + edge._element
                    if w not in location: #if w not in locs, not added into open
                        preds[w] = v# add w:v to preds,
                        s1 = open.add(newcost, w)# add w:newcost to open
                        location[w] =  s1# add w:(elt returned from open) to locs
                        if newcost < open.get_key(location[w]):
                            #update w:v in preds, update w's cost in open to newcost
                            preds[w] = v
                            open.update_key(location[w], newcost)
        if dest_v is not None and v in closed:
            #shortest path from source to destination vertex
            print("Shortest path: from:" ,source_v._element,"to:",dest_v._element ,"is",closed[v][0])
            for key, value in closed.items():
                print(key,value)
        if dest_v is None and v in closed:
            #shortest path from source vertex to all other vertexes in the graph
            result = ""
            for key, values in closed.items():
                result += (key , ": cost/distance(" , values[0], "), predecessor(", values[1],")","\n")
            print(result)
        return closed
        
    elif bool_arg == True:
        #True -> heap implementation APQ 
        open = AdaptablePriorityQueue2()
        closed = {} #keys:Vertex, values: (cost, predecessor)
        location = {}#keys: vertices, values: location in open
        preds = {source_v: None} #no predecessor for source vertex
        s = open.add(0, source_v) #add source_v with key 0 to APQ
        location[source_v] = s #add s to locs, element returned from APQ
        return
    

def dijkstra2(graph:Graph, source_v:Vertex, dest:Vertex, bool_arg:bool):
    """
    Another implementation of Dijkstra 
    Modified from/ Based on: https://www.youtube.com/watch?v=OrJ004Wid4o&t=1955s

    """
    if bool_arg == False:
        #False -> use unsorted list ie. APQ1
        inf = sys.maxsize # number for infinity, initial cost for all vertices
        #get all vertices in graph
        vertices = graph.vertices()
        #create dictionary for storing node data, cost and predecessors
        node_data = {}
        #add all vertices into the node_data dict, with inf cost and no preds
        for vertex in vertices:
            node_data[vertex] = {'cost':inf,'pred':[]}
        #set source vertex to cost of 0, costs nothing to go from the same vertex to itself
        node_data[source_v]['cost'] = 0
        #visited list of vertices visited
        visited = []
        #temp value, vertex being analyzed currently
        temp = source_v
        #for all the vertices in the graph
        for i in range(len(vertices)-1):
            #if the temp -> current vertex being checked is not in the visited list
            if temp not in visited:
                #add the temp to the visited list
                visited.append(temp)
                #created a min heap, helps to find vertex with min edge cost
                open = AdaptablePriorityQueue1()
                #for each vertex connected to the current vertex temp by an edge
                for j in graph._structure[temp]:
                    j_vertex = graph.get_vertex_by_label(j._element)
                    #if the vertex is not in the visited list
                    if j_vertex not in visited:
                        #calculate the new cost of the vertex using the cost of the predecessors
                        cost = node_data[temp]['cost'] + graph._structure[temp][j]._element
                        #if this new cost is smaller then the previously assigned cost
                        if cost < node_data[j_vertex]['cost']:
                            #add the new cost into the node_data
                            node_data[j_vertex]['cost'] = cost
                            #add the predecessors used to get this cost into the predecessors list for that vertex
                            node_data[j_vertex]['pred'] = node_data[temp]['pred'] + list(str(temp._element))
                        #add the vertex and its cost into the min heap
                        open.add(node_data[j_vertex]['cost'], j_vertex)
            temp = open.min()._value#next vertex to be examined
        if dest is not None:
            #print the result from the algorithm
            print("Start vertex:",source_v._element, "Destination:",dest._element,"Shortest distance:",str(node_data[dest]['cost']))
            print("Start vertex:",source_v._element, "Destination:",dest._element,"Shortest path:",str(node_data[dest]['pred'] + list(str(dest._element))))
        elif dest is None:
            print("Start vertex:", source_v._element, "To ->")
            for vertex in node_data:
                v = vertex._element
                print("Vertex:", v, "cost/distance:",node_data[vertex]['cost'], "path/predecessors:", node_data[vertex]['pred'])
    elif bool_arg == True:
        #True -> use heap based ie. APQ2
        inf = sys.maxsize # number for infinity, initial cost for all vertices
        #get all vertices in graph
        vertices = graph.vertices()
        #create dictionary for storing node data, cost and predecessors
        node_data = {}
        #add all vertices into the node_data dict, with inf cost and no preds
        for vertex in vertices:
            node_data[vertex] = {'cost':inf,'pred':[]}
        #set source vertex to cost of 0, costs nothing to go from the same vertex to itself
        node_data[source_v]['cost'] = 0
        #visited list of vertices visited
        visited = []
        #temp value, vertex being analyzed currently
        temp = source_v
        #for all the vertices in the graph
        for i in range(len(vertices)-1):
            #if the temp -> current vertex being checked is not in the visited list
            if temp not in visited:
                #add the temp to the visited list
                visited.append(temp)
                #created a min heap, helps to find vertex with min edge cost
                open = AdaptablePriorityQueue2()
                #for each vertex connected to the current vertex temp by an edge
                for j in graph._structure[temp]:
                    j_vertex = graph.get_vertex_by_label(j._element)
                    #if the vertex is not in the visited list
                    if j_vertex not in visited:
                        #calculate the new cost of the vertex using the cost of the predecessors
                        cost = node_data[temp]['cost'] + graph._structure[temp][j]._element
                        #if this new cost is smaller then the previously assigned cost
                        if cost < node_data[j_vertex]['cost']:
                            #add the new cost into the node_data
                            node_data[j_vertex]['cost'] = cost
                            #add the predecessors used to get this cost into the predecessors list for that vertex
                            node_data[j_vertex]['pred'] = node_data[temp]['pred'] + list(str(temp._element))
                        #add the vertex and its cost into the min heap
                        open.add(node_data[j_vertex]['cost'], j_vertex)
            temp = open.remove_min()#new vertex being examined
        if dest is not None:
            #print the result from the algorithm
            print("Start vertex:",source_v._element, "Destination:",dest._element,"Shortest distance:",str(node_data[dest]['cost']))
            print("Start vertex:",source_v._element, "Destination:",dest._element,"Shortest path:",str(node_data[dest]['pred'] + list(str(dest._element))))
        elif dest is None:
            print("Start vertex:", source_v._element, "To ->")
            for vertex in node_data:
                v = vertex._element
                print("Vertex:", v, "cost/distance:",node_data[vertex]['cost'], "path/predecessors:", node_data[vertex]['pred'])

graph = graphreader("simplegraph1.txt")
vertex1 = graph.get_vertex_by_label(1)
vertex2 = graph.get_vertex_by_label(2)
vertex3 = graph.get_vertex_by_label(3)
vertex4 = graph.get_vertex_by_label(4)
vertex5 = graph.get_vertex_by_label(5)
vertex14 = graph.get_vertex_by_label(14)
#False: unsorted list, True: Heap based
#test for single destination vertex as well as all vertices
# test1 = dijkstra2(graph, vertex1, vertex4, False)
# test2 = dijkstra2(graph, vertex1, None, False)
# test3 = dijkstra2(graph, vertex1, vertex4, True)
# test4 = dijkstra2(graph, vertex1, None, True)


#RUNTIME TESTS:
def runtime_test() -> int:
    """Tests the runtime of a single function, Returns elapsed time"""
    clocktime0 = time.perf_counter()
    dijkstra2(graph, vertex1, vertex4, False)
    clocktime1 = time.perf_counter()
    elapsed_time = clocktime1 - clocktime0
    return "The elapsed time for unsorted list + destination vertex: ",elapsed_time

def runtime_test2() -> int:
    """Tests the runtime of a single function, Returns elapsed time"""
    clocktime0 = time.perf_counter()
    dijkstra2(graph, vertex1, None, False)
    clocktime1 = time.perf_counter()
    elapsed_time = clocktime1 - clocktime0
    return "The elapsed time for unsorted list+ no destination: ",elapsed_time

def runtime_test3() -> int:
    """Tests the runtime of a single function, Returns elapsed time"""
    clocktime0 = time.perf_counter()
    test3 = dijkstra2(graph, vertex1, vertex4, True)
    clocktime1 = time.perf_counter()
    elapsed_time = clocktime1 - clocktime0
    return "The elapsed time for heap based + destination vertex: ",elapsed_time

def runtime_test4() -> int:
    """Tests the runtime of a single function, Returns elapsed time"""
    clocktime0 = time.perf_counter()
    dijkstra2(graph, vertex1, None, True)
    clocktime1 = time.perf_counter()
    elapsed_time = clocktime1 - clocktime0
    return "The elapsed time for heap based + no destination: ",elapsed_time

print(runtime_test())#time for unsorted list + destination vertex:
print(runtime_test2())#time for unsorted list+ no destination:
print(runtime_test3())#time for heap based + destination vertex:
print(runtime_test4())#time for heap based + no destination:


"""RESULTS FROM THE RUNTIME TESTS"""
""" I used graph1 to get theses results as when I run my functions using graph2 I get an indexing/key error,
    and I don't get this error when I use the graph1.
    Unfortunately because of this the results I have do not really give insight into which implementation may be better.
"""
#RESULTS FROM TERMINAL, Printed here:
# |V| = 5; |E| = 7
# Vertices: 1-2-3-4-5-
# Edges: (1--2 : 3.0) (1--3 : 8.0) (1--5 : 5.0) (2--3 : 4.0) (3--4 : 1.0) (3--5 : 6.0) (4--5 : 4.0)
# Start vertex: 1 Destination: 4 Shortest distance: 8.0
# Start vertex: 1 Destination: 4 Shortest path: ['1', '2', '3', '4']
# ('The elapsed time for unsorted list + destination vertex: ', 0.00033460000122431666)
# Start vertex: 1 To ->
# Vertex: 1 cost/distance: 0 path/predecessors: []
# Vertex: 2 cost/distance: 3.0 path/predecessors: ['1']
# Vertex: 3 cost/distance: 7.0 path/predecessors: ['1', '2']
# Vertex: 4 cost/distance: 8.0 path/predecessors: ['1', '2', '3']
# Vertex: 5 cost/distance: 5.0 path/predecessors: ['1']
# ('The elapsed time for unsorted list+ no destination: ', 0.0016425999929197133)
# Start vertex: 1 Destination: 4 Shortest distance: 8.0
# Start vertex: 1 Destination: 4 Shortest path: ['1', '2', '3', '4']
# ('The elapsed time for heap based + destination vertex: ', 0.0004802999901585281)
# Start vertex: 1 To ->
# Vertex: 1 cost/distance: 0 path/predecessors: []
# Vertex: 2 cost/distance: 3.0 path/predecessors: ['1']
# Vertex: 3 cost/distance: 7.0 path/predecessors: ['1', '2']
# Vertex: 4 cost/distance: 8.0 path/predecessors: ['1', '2', '3']
# Vertex: 5 cost/distance: 5.0 path/predecessors: ['1']
# ('The elapsed time for heap based + no destination: ', 0.004265099996700883)