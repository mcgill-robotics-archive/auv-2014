
(cl:in-package :asdf)

(defsystem "sprint1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "Pressure" :depends-on ("_package_Pressure"))
    (:file "_package_Pressure" :depends-on ("_package"))
    (:file "Depth" :depends-on ("_package_Depth"))
    (:file "_package_Depth" :depends-on ("_package"))
    (:file "CVQuery" :depends-on ("_package_CVQuery"))
    (:file "_package_CVQuery" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "Pressure" :depends-on ("_package_Pressure"))
    (:file "_package_Pressure" :depends-on ("_package"))
    (:file "Depth" :depends-on ("_package_Depth"))
    (:file "_package_Depth" :depends-on ("_package"))
    (:file "CVQuery" :depends-on ("_package_CVQuery"))
    (:file "_package_CVQuery" :depends-on ("_package"))
  ))