import os
import sys

counts = 0
check_mode = 0

def check_docs(dirs,suffix_input):
	global counts
	for doc in os.listdir(dirs):
		doc_dir = dirs + '/' + doc
		
		if os.path.isfile(doc_dir):
			if doc[-1] == suffix_input:
				if check_mode == 0:
					os.remove(doc_dir)
					counts += 1
					print 'delete this file: ' + doc_dir + '/'+ doc
				elif check_mode == 1:
					counts += 1
					print 'detect the file: ' + doc_dir + '/'+ doc
				else:
					pass
				pass
		if os.path.isdir(doc_dir):
			check_docs(doc_dir,suffix_input)
			pass
	

def deleteFileSuffix(suffix_input):
	
	current_dir = os.getcwd()
	check_docs(current_dir,suffix_input)
	pass

if __name__ == '__main__':

	delete_str = '~'

	print 'Process suffix: ' + delete_str
	
	if len(sys.argv) == 1:
		print 'Input clearn or check'

	elif sys.argv[1].lower() == 'clean' or sys.argv[1].lower() == 'delete':
		check_mode = 0
		deleteFileSuffix(delete_str)
		print 'Cleaning done'
		print 'Cleaned ' + str(counts) + ' docs'

	elif sys.argv[1].lower() == 'check' or sys.argv[1].lower() == 'detect':
		check_mode = 1
		deleteFileSuffix(delete_str)
		print 'Check done'
		print 'There are ' + str(counts) + ' docs'

	else:
		pass
		
	#if len(sys.argv[1]) == 1 and sys.argv[1] != 'y'
	
	
	
	



