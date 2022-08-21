from gym.envs.registration import register
from importlib_metadata import entry_points

register(
    id='MultiagentReaction-v0',
    entry_points = 'multiagent_reaction.envs:MultiagentReactionEnv'
)