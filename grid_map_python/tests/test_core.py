#!/usr/bin/env python3

import copy
import pickle
import numpy as np
from grid_map import GridMap
np.set_printoptions(precision=2, linewidth=999, suppress=True, sign=' ', formatter={'float': '{:0.2f}'.format})

if __name__ == '__main__':
  # Default constructor
  gm = GridMap()
  gm.setGeometry([10, 11], 0.01, [-27.6, 32.9])
  gm_size = gm.getSize()
  assert gm_size[0]==(10/0.01) and gm_size[1]==(11/0.01)

  # Layer management
  gm.add('a')
  gm.add('b', 10)
  gm.add('c', np.arange(gm_size.prod()).reshape(gm_size))
  assert gm.exists('a')
  assert np.isnan(gm['a'][12,2])
  assert gm.exists('b')
  assert gm['b'][12,2]==10
  assert gm.exists('c')
  assert gm['c'][12,2]==12*gm_size[1]+2
  assert not gm.exists('d')

  # Layer retrieval
  a = gm.get('a')
  a[2,5] = -64
  b = gm.getc('b')
  b[101, 53] = -99

  assert isinstance(a, np.ndarray)
  assert np.all(a.shape==gm_size)
  assert gm['a'][2,5]==-64
  assert np.isnan(gm['a'][2,6])
  assert isinstance(b, np.ndarray)
  assert np.all(b.shape==gm_size)
  assert gm['b'][101, 53]==10 # Should not have changed the value as it b is a copy
  assert gm['b'][101, 53]==10

  gm.erase('b')
  layers = gm.getLayers()
  assert len(layers)==2 and layers[0]=='a' and layers[1]=='c'

  gm.setBasicLayers(['a'])
  basic_layers = gm.getBasicLayers()
  assert len(basic_layers)==1 and basic_layers[0]=='a'
  assert gm.hasBasicLayers()

  # Copy
  gm2 = copy.copy(gm)
  gm3 = copy.copy(gm)
  assert gm2.hasSameLayers(gm)
  assert gm3.hasSameLayers(gm)

  # Pickle
  gm_bytes = pickle.dumps(gm)
  gm4 = pickle.loads(gm_bytes)
  assert gm4.hasSameLayers(gm)

  # Pickle
  gms_ref = {'1': gm, 'a': gm4, 'fiezb': gm2}
  gms_bytes = pickle.dumps(gms_ref)
  gms = pickle.loads(gms_bytes)
  print(len(gms_bytes), len(gm_bytes), gms)
