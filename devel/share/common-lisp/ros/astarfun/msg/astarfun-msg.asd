
(cl:in-package :asdf)

(defsystem "astarfun-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "path_ok" :depends-on ("_package_path_ok"))
    (:file "_package_path_ok" :depends-on ("_package"))
  ))