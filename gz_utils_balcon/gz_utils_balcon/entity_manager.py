from gz.transport13 import Node as Gz_node
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.math7 import Quaterniond

class EntityManager:

    def __init__(self, world_name):
        self.world_name = world_name
        self.gz_node = Gz_node()

    def create_entity(
        self, path_to_sdf, name="unknow", x=0, y=0, z=0, roll=0, pitch=0, yaw=0
    ):
        service = "/world/" + self.world_name + "/create"
        req = EntityFactory()
        req.sdf = f'<?xml version="1.0" ?>\
            <sdf version="1.6">\
            <include>\
            <pose degrees="true">1.0 0.1 0 0 0 50</pose>\
            <name>Object1</name>\
            <uri>{path_to_sdf}</uri>\
            </include>\
            </sdf>'

        q = Quaterniond()
        q.set_from_euler(roll, pitch, yaw)
        req.name = name
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z

        req.pose.orientation.x = q.x()
        req.pose.orientation.y = q.y()
        req.pose.orientation.z = q.z()
        req.pose.orientation.w = q.w()
        return self.gz_node.request(service, req, EntityFactory, Boolean, 5000)

    def delete_entity(self, names_entity):
        for name_entity in names_entity:

            service = "/world/" + self.world_name + "/remove"

            req = Entity()
            req.name = name_entity
            req.type = Entity.MODEL
            self.gz_node.request(service, req, Entity, Boolean, 5000)
