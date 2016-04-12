(define (domain shakey)
    (:requirements :strips :typing :durative-actions :fluents :derived-predicates :equality :conditional-effects)

    (:types
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame

        location - pose                 ; a pose for the robot base
        object_location - location		; object location to push an object to
		doorway_location - location		; doorway location to check if doorway is blocked
		doorway_in_location doorway_out_location - doorway_location      ; locations directed into the doorway or outwards
		room							; a room
		doorway							; a doorway
		
		search_location - location
		
		movable_object          		; an object that can be pushed
		pushable_location - location	; a pushable location of a movable object (in front of the side planes)
    )
    
    (:modules (canDriveToPos ?loc - location
    	      		 conditionchecker canDriveToPos@libshakey_planning_actions.so)
    	      (canPushDistance ?loc - location
    	      		 conditionchecker canPushDistance@libshakey_planning_actions.so)
    	      (canPushToPos ?p ?q - pushable_location ?dest - object_location
    	      		 conditionchecker canPushToPos@libshakey_planning_actions.so)
    	      (costPushDistance ?loc - location
    	       		 cost costPushDistance@libshakey_planning_actions.so)
    	      ;(costDriveToPos ?s ?q - location
			;		cost costDriveToPos@libshakey_planning_actions.so)
    )

    (:constants
        robot_location - location
    )

    (:predicates
        (at-base ?l - location)                 ; location of the base
        (pushed ?o - movable_object)			; records if a movable_object was pushed to its goal
		(doorway-state-known ?d - doorway)		; is the doorway state known?
		(searched ?l - search_location)			; search location searched?
		(object_at ?b - object_location) 		; is an object at this object location?
    )

    (:functions
        (x ?p - pose) - number
        (y ?p - pose) - number
        (z ?p - pose) - number
        ; quaternion orientation
        (qx ?p - pose) - number
        (qy ?p - pose) - number
        (qz ?p - pose) - number
        (qw ?p - pose) - number
        (timestamp ?p - pose) - number      ; unix time in s
        (frame-id ?p - pose) - frameid
		(belongs-to-doorway ?l - doorway_in_location) - doorway
        (location-in-room ?l - location) - room
        ;push informations for a movable object
        (belongs-to-object-location ?o - moveable_object) - object_location
        (belongs-to-movable-object ?u - pushable_location) - movable_object
        (belongs-to-search-location ?o - movable_object) - search_location
        (belongs-to-doorway ?o - movable_object) - doorway
        (push-distance ?o - pushable_location) - number
        (push-cost ?o - pushable_location) - number
    )

    (:durative-action detect-doorway-state
        :parameters (?l - doorway_in_location ?d - doorway)
        :duration (= ?duration 1.0)
        :condition
        (and
            (at start (at-base ?l))
            (at start (= (belongs-to-doorway ?l) ?d))
            (at start (not (doorway-state-known ?d)))
        )
        :effect
        (and
            (at end (doorway-state-known ?d))
         )
    )
    
    (:durative-action drive-through-doorway
        :parameters (?d - doorway ?s - doorway_in_location ?g - doorway_out_location)
        :duration (= ?duration 10.0)
        :condition
        (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            ; dont drive from in to out loc in the same room
            (at start (not (= (location-in-room ?s) (location-in-room ?g))))
            (at start (= (belongs-to-doorway ?s) ?d))
            (at start (= (belongs-to-doorway ?g) ?d))
            (at start (doorway-state-known ?d))
            (at start (not (is-doorway-blocked ?d)))
        )
        :effect
        (and
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (at end (not (doorway-state-known ?d)))
        )
    )

    (:durative-action drive-base
        :parameters (?s - location ?g - location)
        :duration (= ?duration 1000)
        :condition
        (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (can-navigate ?s ?g))
            (at start ([canDriveToPos ?g]))
        )
        :effect
        (and
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
        )
    )
    
    (:durative-action detect-objects
        :parameters (?l - search_location)
        :duration (= ?duration 1.0)
        :condition
        (and
            (at start (at-base ?l))
            (at start (not (searched ?l)))
            (at start (not (not-pushed-objects)))
        )
        :effect
        (and
            (at end (searched ?l))
        )
    )
    
    (:durative-action push-object
		:parameters (?o - movable_object ?u - pushable_location)
		:duration (= ?duration [costPushDistance ?u])
		:condition
		(and
			(at start (at-base ?u))
			(at start (not (pushed ?o)))
			(at start (= (belongs-to-movable-object ?u) ?o))
			(at start ([canPushDistance ?u]))
		)
		:effect
		(and
			(at end (pushed ?o))
		)
	)
	
	(:durative-action push-object-to-pos
		:parameters (?o - movable_object ?p - pushable_location ?q - pushable_location ?b - object_location)
		:duration (= ?duration 1000)
		:condition
		(and
			(at start (not (pushed ?o)))
			(at start (= (location-in-room ?p) (location-in-room ?b)))
			(at start (= (location-in-room ?q) (location-in-room ?b)))
			(at start (not (= ?p ?q)))
			(at start (not (exists (doorway ?d) (= (belongs-to-doorway ?o) ?d))))
			(at start (not (exists (object_location ?t) (= (belongs-to-object-location ?o) ?t))))
			(at start (at-base ?p))
			(at start ([canPushToPos ?p ?q ?b]))
		)
		:effect
		(and
			(at end (pushed ?o))
			(at end (assign (belongs-to-object-location ?o) ?b))
			(at end (object_at ?b))
		)
	)
	
    (:derived
        (can-navigate ?s ?g - location)
        (and
            (= (location-in-room ?s) (location-in-room ?g))
            (not (is-doorway-out-location ?g))
            (not (= ?g robot_location))
        )
    )
    
    (:derived
        (is-doorway-out-location ?l - doorway_location)
        (exists (?l2 - passage_out_location) (= ?l ?l2))
    )
    
    (:derived
        (is-doorway-blocked ?d - doorway)
        (exists (?o - movable_object) 
			(and
				(= (belongs-to-doorway ?o) ?d)
				(not (pushed ?o))
			)
		)
    )
    
    (:derived
        (not-pushed-objects)
        (exists (?o - movable_object) 
			(and
				(not (pushed ?o))
			)
		)
    )
    
    (:derived
        (object-at-location ?b - object_location)
        ; Not already assigned to a doorway(_block_location) and in same room
		(or	
			(object_at ?b)
			(forall (?o - movable_object)
				(or 
					(exists (?d doorway) (= (belongs-to-doorway ?o) ?d))
					(exists (?t object_location) (= (belongs-to-object-location ?o) ?t))
					(forall (?p - pushable_location) 
						(or 
							(not (= (location-in-room ?b) (location-in-room ?p)))
							(not (= (belongs-to-movable-object ?p) ?o))
						)
					)
				)
			)
		)
	)
)

