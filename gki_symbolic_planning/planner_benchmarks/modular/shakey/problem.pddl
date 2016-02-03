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
    obj3 - movable_object
    obj1_push_loc1 - pushable_location
    obj2_push_loc1 - pushable_location
    obj3_push_loc1 - pushable_location	
    sr_loc1 - search_location
    sr_loc2 - search_location
    sr_loc3 - search_location
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
    (= (belongs-to-search_location obj1) sr_loc1)
    (= (location-in-room obj1_push_loc1) room1)
    (= (location-in-room sr_loc1) room1)
    (= (belongs-to-movable-object obj2_push_loc1) obj2)
    (= (belongs-to-search_location obj2) sr_loc2)
    (= (location-in-room obj2_push_loc1) room2)
    (= (location-in-room sr_loc2) room2)
    (= (location-in-room sr_loc3) room1)
    (= (location-in-room obj3_push_loc1) room1)
    (= (belongs-to-doorway obj3) pass1)
    (= (belongs-to-movable-object obj3_push_loc1) obj3)
  )
  (:goal (and
	(forall (?o - movable_object) (pushed ?o))
	(forall (?l - search_location) (searched ?l))
	)
  )
)
