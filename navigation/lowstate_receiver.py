"""LowState subscription with a persistent wait set, drained by one receive thread."""
from cyclonedds.core import InstanceState, ReadCondition, SampleState, ViewState, WaitSet
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader, InvalidSample
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class LowStateSubscription:
    def __init__(self, domain_id=0):
        # Reuse the domain configuration established by ChannelFactoryInitialize.
        self.participant = DomainParticipant(domain_id)
        self.topic = Topic(self.participant, 'rt/lowstate', LowState_)
        self.reader = DataReader(self.participant, self.topic)
        self.condition = ReadCondition(
            self.reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead)
        self.waitset = WaitSet(self.participant)
        self.waitset.attach(self.condition)

    def Read(self, timeout=.05):
        samples = self.reader.take(1, condition=self.condition)
        if not samples:
            self.waitset.wait(duration(seconds=timeout))
            samples = self.reader.take(1, condition=self.condition)
        if samples and not isinstance(samples[0], InvalidSample):
            return samples[0]
        return None

    def Close(self):
        self.waitset.detach(self.condition)
        self.condition = self.waitset = self.reader = self.topic = self.participant = None
