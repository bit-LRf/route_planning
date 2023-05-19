;; Auto-generated. Do not edit!


(when (boundp 'astarfun::path_ok)
  (if (not (find-package "ASTARFUN"))
    (make-package "ASTARFUN"))
  (shadow 'path_ok (find-package "ASTARFUN")))
(unless (find-package "ASTARFUN::PATH_OK")
  (make-package "ASTARFUN::PATH_OK"))

(in-package "ROS")
;;//! \htmlinclude path_ok.msg.html


(defclass astarfun::path_ok
  :super ros::object
  :slots (_path_ok ))

(defmethod astarfun::path_ok
  (:init
   (&key
    ((:path_ok __path_ok) nil)
    )
   (send-super :init)
   (setq _path_ok __path_ok)
   self)
  (:path_ok
   (&optional __path_ok)
   (if __path_ok (setq _path_ok __path_ok)) _path_ok)
  (:serialization-length
   ()
   (+
    ;; bool _path_ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _path_ok
       (if _path_ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _path_ok
     (setq _path_ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get astarfun::path_ok :md5sum-) "276f6ed1d160276c34d8bfda3a611a63")
(setf (get astarfun::path_ok :datatype-) "astarfun/path_ok")
(setf (get astarfun::path_ok :definition-)
      "bool path_ok
")



(provide :astarfun/path_ok "276f6ed1d160276c34d8bfda3a611a63")


