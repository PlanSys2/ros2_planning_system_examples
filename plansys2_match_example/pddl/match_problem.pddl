(define (problem fixfuse)
  (:domain matchcellar)

  (:objects
    match1 match2 - match
    fuse1 fuse2 - fuse
  )

  (:init
    (unused match1)
    (unused match2)
    (handfree)
  )

  (:goal
    (and
      (mended fuse1)
      (mended fuse2)
    )
  )

  (:metric minimize (total-time))
)
