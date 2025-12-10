PRIORITY_LIST = ["TurtleBot3Burger_1", "TurtleBot3Burger_2", "TurtleBot3Burger_3"]
full_ack_set= {x for x in PRIORITY_LIST if x != 'TurtleBot3Burger_1'}
ack_set={'TurtleBot3Burger_2', 'TurtleBot3Burger_3'}

print(f'{full_ack_set=}')
print(f'{ack_set=}')
print(f'{ack_set == full_ack_set=}')