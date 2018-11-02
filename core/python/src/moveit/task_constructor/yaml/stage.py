#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains support for saving a stage's current properties
to YAML and loading them later.
"""

__author__ = 'Jan Ebert'

from yaml import add_multi_representer, add_multi_constructor

from moveit.task_constructor import core
from moveit.task_constructor import stages
from moveit.task_constructor.yaml import utils

BASE_CLASS = core.Stage
"""Base class to use for the multi representer."""

TAG_PREFIX = u'MTCStage.'
"""How to prefix stages for the multi constructor.

Arbitrary; but changing this value will break compatibility.
"""


def _represent_stage(dumper, stage):
    """Return a stage in YAML format.

    Used as a PyYAML `multi_representer` for the base class given by
    `BASE_CLASS`.
    """
    return dumper.represent_mapping(TAG_PREFIX + type(stage).__name__,
            {'name': stage.name, 'properties': dict(stage.properties)})


def _construct_stage(loader, tag_suffix, node):
    """Construct a stage from the given PyYAML node.

    Used as a PyYAML `multi_constructor` for the tag prefix given by
    `TAG_PREFIX`.
    """
    try:
        stage_class = getattr(stages, tag_suffix)
    except AttributeError as ex:
        try:
            stage_class = getattr(core, tag_suffix)
        except AttributeError:
            utils.raisefrom(ex, utils.TypeNotFoundError(tag_suffix))
    stage = stage_class()

    attributes = loader.construct_mapping(node)
    stage.name = attributes['name']
    # set stage properties
    utils.properties_from_dict(stage, attributes['properties'], skip_none=True)
    return stage


add_multi_representer(BASE_CLASS, _represent_stage, Dumper=utils.Dumper)
add_multi_constructor(TAG_PREFIX, _construct_stage, Loader=utils.Loader)

