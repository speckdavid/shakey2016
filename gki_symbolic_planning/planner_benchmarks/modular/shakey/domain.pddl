(define (domain shakey)
    (:requirements :strips :typing :durative-actions :fluents :derived-predicates :equality)

    (:types
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame

        location - pose                 ; a pose for the robot base
		doorway_location - location		; doorway location to check if doorway is blocked
		doorway_in_location doorway_out_location - doorway_location      ; locations directed into the doorway or outwards
		room							; a room
		doorway							; a doorway
		
		movable_object          		; an object that can be pushed
		pushable_location - location	; a pushable location of a movable object (in front of the side planes)
    )

    (:constants
        robot_location - location
    )

    (:predicates
        (at-base ?l - location)                 ; location of the base
        (pushed ?o - movable_object)			; records if a movable_object was pushed to its goal
		(doorway-free ?d - doorway)				; is the doorway is free?
		(doorway-state-known ?d - doorway)		; is the doorway state known?
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
		(belongs-to-doorway ?l - doorway_location) - doorway
        (location-in-room ?l - location) - room
        ;push informations for a movable object
        (belongs-to-movable-object ?u - pushable_location) - movable_object
        (push-distance ?o - movable_object) - number
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
    
    (:durative-action unblock-doorway
        :parameters (?l - doorway_in_location ?d - doorway)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (at-base ?l))
            (at start (= (belongs-to-doorway ?l) ?d))
            (at start (doorway-state-known ?d))
            (at start (not (doorway-free ?d)))
        )
        :effect
        (and
            (at end (doorway-free ?d))
            (at start (not (doorway-state-known ?d)))	; necessary if a ghost blocks again a doorway or it fails
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
            (at start (doorway-free ?d))
        )
        :effect
        (and
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (at end (not (door-state-known ?d)))
            ; set robot_location to goal room
        )
    )

    (:durative-action drive-base
        :parameters (?s - location ?g - location)
        :duration (= ?duration 1000.0)
        :condition
        (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            ;(at start (can-navigate ?s ?g))
        )
        :effect
        (and
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
        )
    )
    
    (:durative-action push-object
		:parameters (?o - movable_object ?u - pushable_location)
		:duration (= ?duration 1000.0)
		:condition
		(and
			(at start (at-base ?u))
			(at start (not (pushed ?o)))
			(at start (= (belongs-to-movable-object ?u) ?o))
		)
		:effect
			(at end (pushed ?o))
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
)

