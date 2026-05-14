# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    Flink = 0.0

    # Example: linear spring
    # k = 1000 #N/m
    # Z0= 0.1  #m
    # Flink = k*(Z-Z0)
    """
    spring1 = mbs_data.link_id['Spring1']
    spring2 = mbs_data.link_id['Spring2']
    spring3 = mbs_data.link_id['Spring3']
    spring4 = mbs_data.link_id['Spring4']

    if identity == spring1 or identity == spring2 or identity == spring3 or identity == spring4:
        K = mbs_data.user_model['springs']['K']
        D = mbs_data.user_model['springs']['D']
        L0 = mbs_data.user_model['springs']['L0']

        Flink = K*(Z-L0) + D*Zd

    #print(Flink)"""

    return Flink
