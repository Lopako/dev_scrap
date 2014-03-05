from copy import deepcopy
from pprint import pprint
import rospy
import diagnostic_msgs.msg


class DiagCounter(object):

    def __init__(self, watch_str = "Joint 0"):
        self._watch_name = watch_str
        self.last_time = rospy.Time.now()
        self.dur_secs = rospy.Duration(0)
        self.last_count = 0
        self.last_msg = diagnostic_msgs.msg.DiagnosticArray()
        self._calls = dict()
        self._msgs = dict()
        self._times = list()
        self._diffs = list()
        self._avg = 0
#         self.all_last_msg = diagnostic_msgs.msg.DiagnosticArray()
        self._sub = rospy.Subscriber('/diagnostics',
                        diagnostic_msgs.msg.DiagnosticArray, self.print_code)


    def print_code(self, m):
        time_now = rospy.Time.now()

        self.dur_secs = time_now - self.last_time
        self.last_count = len(m.status)
        self.last_msg = deepcopy(m)

        caller = m._connection_header['callerid']

        if not caller in self._calls:
            self._calls[caller] = {'calls': 0, 'msgs': 0, 'names': 0}
            self._msgs[caller] = dict()

        self._calls[caller]['calls'] = self._calls[caller]['calls'] + 1
        self._calls[caller]['msgs'] = self._calls[caller]['msgs'] + self.last_count
#         print caller
        print ""

        clear_counts = False
        for stat in m.status:
            if (stat.name not in self._msgs[caller]
                    or not self._msgs[caller][stat.name] > 0):
                self._msgs[caller][stat.name] = 1
                self._calls[caller]['names'] = self._calls[caller]['names'] + 1

#             self._calls[caller]['names'] = len(self._msgs[caller])
            if self._watch_name in stat.name:
                clear_counts = True
                self._times.append(time_now)
                print(self.dur_secs)
                self.last_time = time_now
                if len(self._times) > 1:
                    self._diffs.append(self.dur_secs.to_sec())

                print m
                if self.last_count > 1:
                    print "!!!!!!!! More than one message: %d stati" % (len(m.status),)

        print self.dur_secs.secs
#         pprint(self._calls)
        for call_id in self._calls.keys():
            print '{:<14}: '.format(call_id),
            pprint(self._calls[call_id])
        print "Diffs:",
        print self._diffs

        if clear_counts:
            self._calls = dict([(key,{'calls':0, 'msgs':0, 'names':0}) for key in self._calls.keys()])
#             self._msgs = dict(zip(self._msgs.keys(),[dict()]*len(self._msgs.keys())))
            for call_id in self._msgs.keys():
                self._msgs[call_id] = dict(zip(
                    self._msgs[call_id].keys(),[0]*len(self._msgs[call_id].keys())))


if __name__ == '__main__':
    rospy.init_node('rob_test_diag', anonymous=True)
    watch_str = rospy.get_param('~watch', "right_w2 voltage")
    diag_count = DiagCounter(watch_str)

    i = 0
    while not rospy.is_shutdown():
#         print i
        i = i+1
        rospy.sleep(1)
