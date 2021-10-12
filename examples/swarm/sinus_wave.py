import math
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

phase = 0

uris = [
    'radio://0/20/2M/E7E7E7E701',
    'radio://0/20/2M/E7E7E7E702',
    'radio://0/20/2M/E7E7E7E703',
    'radio://0/20/2M/E7E7E7E704',
    #'radio://0/20/2M/E7E7E7E708',
]


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def get_height(scf):
    pos = uris.index(scf.cf.link_uri)
    where = (pos + phase) % 6
    return 1.2 + 0.5 * math.sin(where * math.pi / 6.0)


def take_off(scf):
    h = get_height(scf)
    print(f'uri: {scf.cf.link_uri} takeoff: {h}')
    scf.cf.high_level_commander.takeoff(h, h * 2.0)
    time.sleep(h * 2.0)


def wave(scf, pos):
    h = get_height(scf)
    print(f'uri: {scf.cf.link_uri} goto: {pos.x}, {pos.y}, {h}')
   # scf.cf.high_level_commander.go_to(pos.x, pos.y, h, 0, 0.4)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.reset_estimators()

        positions = swarm.get_estimated_positions()
        uris.sort(key=lambda uri: positions[uri].y)

        swarm.sequential(take_off)

        while phase < 7 * 6:
            swarm.parallel_safe(wave, args_dict=positions)
            time.sleep(0.3)
            phase += 1

        swarm.sequential(lambda scf: scf.cf.high_level_commander.land(0, 2.0))
