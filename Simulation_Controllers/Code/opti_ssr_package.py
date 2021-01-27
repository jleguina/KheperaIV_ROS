"""
A python module to connect to Optitrack optical tracking system
and receive data from it.
"""

import numpy as np
import pyquaternion  # for handling quaternions
import optirx as rx

class OptiTrackClient:
    """
    Connect to Optitrack systems and Motive software and receive data,
    including rigid body position and orientation, from it.
    By default, it connects to Optitrack software Motive on the same machine.

    Attributes
    ----------
    unicast_ip : str, optional
        IP of the Motive software to establish a unicast connection to.
        By default, no unicast connection is established.
    multicast_ip : str, optional
        Multicast address to connect to.
    port : int, optional
        Port of the Motive network interface.
    natnet_version : tuple, optional
        Version number of the NatNetSDK to use.
    """

    def __init__(self, unicast_ip=None, multicast_ip="239.255.42.99", port=1511, natnet_version=(2, 10, 0, 0)):
        self._dsock = rx.mkdatasock(ip_address=unicast_ip, multicast_address=multicast_ip, port=port)
        self._natnet_version = natnet_version

    def get_packet_data(self, packet_types=[rx.SenderData, rx.ModelDefs, rx.FrameOfData]):
        """
        Receive desired packet data.

        based on optirx-demo.py
        source: https://bitbucket.org/astanin/python-optirx

        Parameters
        ----------
        packet_types : list, optional
            Types of the packets to be returned.

        Returns
        -------
        packet : list
            Received packets of desired type.

        """
        while True:
            data = self._dsock.recv(rx.MAX_PACKETSIZE)
            packet = rx.unpack(data, version=self._natnet_version)
            if not packet_types or type(packet) in packet_types:
                return packet


    def get_rigid_body(self, rb_id=0):
        """
        Receive rigid body position, orientation and time data.

        Parameters
        ----------
        rb_id : int, optional
            ID of the rigid body to receive data from.

        Returns
        -------
        position : numpy array
            Rigid body position packet data of the desired rigid body.
            Consists of x, y, z coordinates of Motive's coordinate system.
        orientation : list
            List of rigid body orientation data in quaternion representation.
        time_data : list
            List of time data consisting of frame mumber, timestamp and latency packet data.

        """
        packet = self.get_packet_data()

        position = np.array(packet.rigid_bodies[rb_id].position)
        qx, qy, qz, qw = tuple(packet.rigid_bodies[rb_id].orientation)
        orientation = Quaternion(a=qw, b=qx, c=qy, d=qz)
        time_data = (packet.frameno, packet.timestamp, packet.latency)

        return position, orientation, time_data


class Quaternion(pyquaternion.Quaternion):
    """Work-around until pull request for original packages is accepted
    https://github.com/KieranWynn/pyquaternion/pull/2
    """

    @property
    def yaw_pitch_roll(self):
        """
        Get the equivalent yaw-pitch-roll angles aka intrinsic Tait-Bryan
        angles following the z-y'-x'' convention

        Returns
        -------
        yaw: double
            rotation angle around the z-axis in radians, in the range
            `[-pi, pi]`
        pitch: double
            rotation angle around the y'-axis in radians, in the range
            `[-pi/2, -pi/2]`
        roll: double
            rotation angle around the x''-axis in radians, in the range
            `[-pi, pi]`

        Note
        ----
        This feature only makes sense when referring to a unit quaternion.
        Calling this method will implicitly normalise the Quaternion object
        to a unit quaternion if it is not already one.
        """

        self._normalise()
        yaw = np.arctan2(2*(self.q[0]*self.q[3] + self.q[1]*self.q[2]),
            1 - 2*(self.q[2]**2 + self.q[3]**2))
        pitch = np.arcsin(2*(self.q[0]*self.q[2] - self.q[3]*self.q[1]))
        roll = np.arctan2(2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]),
            1 - 2*(self.q[1]**2 + self.q[2]**2))

        return yaw, pitch, roll


# if __name__ == '__main__':
#     a = OptiTrackClient()
#     print("Checkpoint1")
#     c,v,b = a.get_rigid_body()
#     print(c)