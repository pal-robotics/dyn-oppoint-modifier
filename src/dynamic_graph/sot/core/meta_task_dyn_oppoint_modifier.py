from dynamic_graph.sot.core.meta_task_6d import *
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier
from dynamic_graph.sot.core.matrix_util import matrixToTuple
import numpy

class MetaTaskDynamicOppoint(MetaTask6d):
	def __init__(self,name,dyn,opPoint,opPointRef='right-wrist'):
		MetaTask6d.__init__(self, name, dyn, opPoint, opPointRef)
		offset= numpy.eye(4)
		offset[0:3,3] = (0.0,0.0,0.0)
		self.task.opmodif = matrixToTuple(offset)
		self.opPointModif.activ = True

	def createOpPointModif(self):
		print 'child class modif'
		self.opPointModif = DynamicOpPointModifier('dynopmodif'+self.name)
		plug(self.dyn.signal(self.opPoint),self.opPointModif.signal('positionIN'))
		plug(self.dyn.signal('J'+self.opPoint),self.opPointModif.signal('jacobianIN'))
		self.opPointModif.activ = False
