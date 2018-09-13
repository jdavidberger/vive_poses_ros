import openvr
import rospy
import sys
import tf

from geometry_msgs.msg import PoseStamped

class VivePosesRosPublisher:
    def __init__(self, topic_prefix):
        self.vr_system = openvr.init(openvr.VRApplication_Utility)
        self.topic_prefix = topic_prefix
        self.publishers = {};
        
    def publisher(self, name):
        if name in self.publishers:
            return self.publishers[name]
        self.publishers.update({name: rospy.Publisher(self.topic_prefix + name, PoseStamped, queue_size=1000) })
        return self.publishers[name]
                
    def idx_to_name(self, idx):
        if idx == 0:
            return "HMD"
        return "obj" + str(idx)
        
    def run(self):
        seq=0
        device_poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
        rate = rospy.Rate(500)
        
        while not rospy.is_shutdown():
            self.vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseRawAndUncalibrated, 0, openvr.k_unMaxTrackedDeviceCount, device_poses)
            rate.sleep()            
            for i in range(0, len(device_poses)):
                pose = device_poses[i]

                if pose.bPoseIsValid:
                    pose_msg = PoseStamped()
                    pose_msg.header.seq = seq
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "openvr_world"
                    mat=pose.mDeviceToAbsoluteTracking

                    mat44 = [
                        [ mat[0][0], mat[0][1], mat[0][2], mat[0][3] ],
                        [ mat[1][0], mat[1][1], mat[1][2], mat[1][3] ],
                        [ mat[2][0], mat[2][1], mat[2][2], mat[2][3] ],
                        [ 0, 0, 0, 1 ]
                    ]
                        
		    pose_msg.pose.position.x = mat[0][3]
		    pose_msg.pose.position.y = mat[1][3]
		    pose_msg.pose.position.z = mat[2][3]

                    q=tf.transformations.quaternion_from_matrix(mat44)
                    pose_msg.pose.orientation.w = q[3]
                    pose_msg.pose.orientation.x = q[0]
                    pose_msg.pose.orientation.y = q[1]
                    pose_msg.pose.orientation.z = q[2]
                    seq = seq + 1

                    self.publisher(self.idx_to_name(i)).publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node("VivePosesRos", anonymous=True)
    topic_prefix = ''
    if len(sys.argv) > 1:
        topic_prefix = sys.argv[1]
    publisher = VivePosesRosPublisher(topic_prefix);
    publisher.run()
    pass
