#!/usr/bin/env python
import rospy
import re
import threading
import numpy as np
from geometric_error.msg import Error 
from geometric_error.msg import ErrorInfo 

n1 = False
n2 = False
trans_mat1 = np.ones((2, 2))
trans_mat2 = np.ones((2, 2))
scale1 = 1
scale2 = 1
cam_num = 2
error1 = []
error2 = []
error1_dim = []
error2_dim = []
received = [False, False]

def normalize(errors, dim):

	global trans_mat1
	global scale1
	global trans_mat2
	global scale2
	global n1
	global n2
	if len(errors) == 0:
		return []
	
	num_cols = len(errors) / dim
	num_rows = dim
	error_mat = np.matrix(np.zeros((num_rows, num_cols)))
	norm_mat = np.matrix(np.zeros((num_rows, num_cols)))
	# print "num cols: ", num_cols
	# print "num rows: ", num_rows
	if num_cols == 1:
		return errors

	# Translation to get centroid at origin and scale to make distance from 
	# origin srtq(2).
	if num_rows == 1:
		for i in range(len(errors)):
			error_mat[0, i] = errors[i]
		
		if(not n1):
			trans = -(error_mat.sum() / num_cols);
			trans_mat1 = np.multiply(trans, np.matrix(np.ones((num_rows, num_cols))))
		norm_mat = error_mat + trans_mat1;
		if(not n1):
			centroid = norm_mat.sum() / num_cols;
			scale1 = (num_cols * np.sqrt(2)) / np.absolute(norm_mat).sum();
		norm_mat = np.multiply(50 * scale1, norm_mat);
		
		dist = np.absolute(norm_mat).sum() / num_cols;
		'''print "Translation: " + str(trans)
		print "Centroid: " + str(centroid)
		print "Distance: " + str(dist) '''
		n1 = True
	
	if num_rows == 2: 
		for i in range(0, len(errors), 2):
			error_mat[0, i/2] = errors[i]
			error_mat[1, i/2] = errors[i + 1]
		
		if(not n2):
			trans_x = -(error_mat[0, :].sum() / num_cols)
			trans_y = -(error_mat[1, :].sum() / num_cols)
			trans_mat2 = np.matrix(np.ones((num_rows, num_cols)))
			trans_mat2[0, :] = np.multiply(trans_mat2[0, :], trans_x)
			trans_mat2[1, :] = np.multiply(trans_mat2[1, :], trans_y)
		norm_mat = error_mat + trans_mat2
		if(not n2):
			centroid_x = norm_mat[0, :].sum() / num_cols
			centroid_y = norm_mat[1, :].sum() / num_cols
			scale2 = (num_cols * np.sqrt(2)) / np.linalg.norm(norm_mat, axis=0).sum()
		norm_mat = np.multiply(50 * scale2, norm_mat)
		
		dist = np.linalg.norm(norm_mat, axis=0).sum() / num_cols
		
		n2 = True
		
		'''print 'Translation: (' + str(trans_x) + ', ' + str(trans_y) + ')'
		print 'Centroid: (' + str(centroid_x) + ', ' + str(centroid_y) + ')' 
		print 'Distance: ' + str(dist) '''

	return norm_mat.getA1().tolist()

def divide_errors(errors, error_dims):

	errors_1d = []
	errors_2d = []
	
	j = 0
	i = 0
	while i in range(len(error_dims)):
		if(error_dims[i] == 1):
			errors_1d.append(errors[j])
			j = j + 1
		else:
			errors_2d.append(errors[j])
			errors_2d.append(errors[j + 1])
			j = j + 2
		i = i + 1
 
	return errors_1d, errors_2d

def interface1_cb(msg):
		global received
		global error1
		global error1_dim
		global error2
		global error2_dim
		global cam_num
		
		if len(msg.error) == 0:
				return
				
		lock.acquire()
		received[0] = True
		
		error1[:] = []
		error1_dim[:] =[]
		for i in range(len(msg.error)):
			error1.append(msg.error[i])
			
		for i in range(len(msg.error_dim)):
			error1_dim.append(msg.error_dim[i])
		
		if (cam_num == 2) and (False in received):
			lock.release()
			return
		else:
			received = [False, False]
			errors_1d, errors_2d = divide_errors(error1 + error2, error1_dim + error2_dim)
			# print "Before 1D: "
			# print errors_1d
			# print "Before 2D: "
			# print errors_2d
			norm_error = normalize(errors_1d, 1) + normalize(errors_2d, 2)
			# print "Norm Error: "
			# print norm_error
			# print 'Error: '
			# print(error1 + error2)
			# error_pub.publish(error1 + error2)
			error_pub.publish(norm_error)
			error1 = []
			error2 = []
			error1_dim = []
			error2_dim = []
			lock.release()

def interface2_cb(msg):
		
		global received
		global error1
		global error1_dim
		global error2
		global error2_dim
		global cam_num
		
		if len(msg.error) == 0:
				return
		lock.acquire()
		received[1] = True
		
		error2[:] = []
		error2_dim[:] = []
		for i in range(len(msg.error)):
			error2.append(msg.error[i])
			
		for i in range(len(msg.error_dim)):
			error2_dim.append(msg.error_dim[i])
		
		if False in received:
			lock.release()
			return
		else:
			received = [False, False]
			errors_1d, errors_2d = divide_errors(error1 + error2, error1_dim + error2_dim)
			# print "Before 1D: "
			# print errors_1d
			# print "Before 2D: "
			# print errors_2d
			norm_error = normalize(errors_1d, 1) + normalize(errors_2d, 2)
			# print "After: "
			# print norm_error
			# print 'Error: '
			# print(error1 + error2)
			# error_pub.publish(error1 + error2)
			error_pub.publish(norm_error)
			error1 = []
			error2 = []
			error1_dim = []
			error2_dim = []
			lock.release()

if __name__ == "__main__":

		lock = threading.Lock()

		# Find out how many cameras we are using.    
		rospy.init_node("error_grouper")

		T = rospy.get_published_topics()
		text = ''.join(str(r) for t in T for r in t)
		if re.match('.*?(cam2/camera).*?', text) != None:
			cam_num = 2
		else:
			cam_num = 1

		rospy.Subscriber("/cam1/ErrorInfo", ErrorInfo, interface1_cb)
		if(cam_num == 2):
			rospy.Subscriber("/cam2/ErrorInfo", ErrorInfo, interface2_cb)
		
		error_pub = rospy.Publisher("image_error", Error, queue_size=1)

		rospy.spin()
