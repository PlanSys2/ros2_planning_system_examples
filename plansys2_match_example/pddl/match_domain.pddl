(define (domain matchcellar)
  (:requirements :durative-actions :typing)

  (:types match fuse - object)

  (:predicates
    (light ?m - match)
    (handfree)
    (unused ?m - match)
    (mended ?f - fuse)
  )

  (:durative-action light_match
    :parameters (?m - match)
    :duration (= ?duration 8)
    :condition (and (at start (unused ?m))
                    (over all (light ?m)))
    :effect (and (at start (not (unused ?m)))
                 (at start (light ?m))
                 (at end (not (light ?m))))
  )

  (:durative-action mend_fuse
    :parameters (?f - fuse ?m - match)
    :duration (= ?duration 5)
    :condition (and (at start (handfree))
                    (over all (light ?m)))
    :effect (and (at start (not (handfree)))
                 (at end (mended ?f))
                 (at end (handfree)))
  )
)
