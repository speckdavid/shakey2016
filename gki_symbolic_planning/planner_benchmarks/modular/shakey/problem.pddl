(define (problem p01)
  (:domain shakey)
  (:objects
    /map - frameid
    start_pos_room1 - location
    final_pos_room2 - location
    room1 - room
    room2 - room
    obj1 - movable_object
    obj2 - movable_object
    obj1_push_loc1 - pushable_location
    obj2_push_loc1 - pushable_location
    pass1 - doorway
    pass1_in_loc_room1 - doorway_in_location
    pass1_out_loc_room2 - doorway_out_location
  )
  (:init
    (= (location-in-room start_pos_room1) room1)
    (= (location-in-room final_pos_room2) room2)
    (at-base start_pos_room1)
    (= (belongs-to-doorway pass1_in_loc_room1) pass1)
    (= (belongs-to-doorway pass1_out_loc_room2) pass1)
    (= (location-in-room pass1_in_loc_room1) room1)
    (= (location-in-room pass1_out_loc_room2) room2)
    (= (belongs-to-movable-object obj1_push_loc1) obj1)
    (= (location-in-room obj1_push_loc1) room1)
    (= (belongs-to-movable-object obj2_push_loc1) obj2)
    (= (location-in-room obj2_push_loc1) room2)
  )
  (:goal (and
	(pushed obj1)
	(pushed obj2)
	)
  )
)
