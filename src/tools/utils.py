import numpy as np

def find_closest_element_sorted(sorted_array, target):
	"""
	Finds the closest element to the target in a sorted array.

	Parameters:
	- sorted_array: A list of sorted numbers (ascending order).
	- target: The target value to find the closest element to.

	Returns:
	- The closest value in the sorted_array to the target.
	"""
	if len(sorted_array) == 0:
		return None, None

	# Edge cases: target is less than the first element or greater than the last element
	if target <= sorted_array[0]:
		return sorted_array[0], 0
	if target >= sorted_array[-1]:
		return sorted_array[-1], len(sorted_array) - 1

	# Binary search initialization
	left, right = 0, len(sorted_array) - 1
	while left <= right:
		mid = (left + right) // 2
		# If target is found
		if sorted_array[mid] == target:
			return sorted_array[mid], mid
		# Narrowing down the search range
		if target < sorted_array[mid]:
			right = mid - 1
		else:
			left = mid + 1

	# At this point, 'right' is less than 'target' and 'left' is greater than 'target'
	# We find the closest of the two
	if abs(sorted_array[right] - target) <= abs(sorted_array[left] - target):
		return sorted_array[right], right
	else:
		return sorted_array[left], left

def filter_sensor(sensor_name, sensor_frameid_dict, rostopic_msg_frameid_dict):
	filter_sensor_frameid_dict = {key: value for key, value in sensor_frameid_dict.items() if sensor_name not in key}
	filter_rostopic_msg_frameid_dict = {key: value for key, value in rostopic_msg_frameid_dict.items() if sensor_name not in key}
	return filter_sensor_frameid_dict, filter_rostopic_msg_frameid_dict

if __name__ == '__main__':
	target = 10
	sorted_array = [1, 2, 4, 5, 6, 8, 9]
	closest_element, closest_id = find_closest_element_sorted(sorted_array, target)
	print(f"The closest element to {target} in the array is {closest_element} - id: {closest_id}.")