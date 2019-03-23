import heapq

class Priority_Queue:
    def __init__(self):
        self.data = []
        self.entry_finder = {}

    def empty(self):
        return len(self.entry_finder)==0

    def in_queue(self,item_config):
        return item_config in self.entry_finder

    def add_item(self,item, priority, step):
        entry = [priority,step, item]
        heapq.heappush(self.data,entry)
        self.entry_finder[item.config] = entry

    def decrease_key(self, item, priority, step):
        item_in_data = self.entry_finder[item.config]
        if priority < item_in_data[0]:
            item_in_data[-1] = None
            new_item_in_data = [priority, step, item]
            heapq.heappush(self.data,new_item_in_data)
            self.entry_finder[item.config] = new_item_in_data

    def pop_item(self):
        while self.data:
            priority, step, item = heapq.heappop(self.data)
            if item != None:
                del self.entry_finder[item.config]
                return item
        # raise KeyError('pop from an empty priority queue')

class Item:
    def __init__(self, config,step):
        self.config = config
        self.step = step


def main():
    pq = Priority_Queue()
    item_1 = Item('config 1',1)
    item_2 = Item('config 2',2)
    item_3 = Item('config 3',3)
    item_2_1 = Item('config 2',4)
    item_3_1 = Item('config 3',5)

    pq.add_item(item_1,1,1)
    pq.add_item(item_2,4,2)
    pq.add_item(item_3,2,3)
    pq.decrease_key(item_2_1,2,4)
    pq.decrease_key(item_3_1,1,5)
    count = 0
    while not pq.empty():
        count +=1
        print(count)
        item =pq.pop_item()
        print(item.config, item.step)




if __name__ == '__main__':

    main()
