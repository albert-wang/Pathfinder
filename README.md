Pathfinder
=================

A Heirarchal Pathfinding algorithm, based on [http://webdocs.cs.ualberta.ca/~mmueller/ps/hpastar.pdf](This paper).
Implemented with clearance values for different terrain types.


Goals
----------------

* Generate near optimal paths, no more than 10% deviation from a true optimal path
* Return incomplete paths so an actor can move before pathfinding is 100% done
* No allocation of memory in the main algorithm
* Trivially parallelizable
* Paths respond to terrain changes
* Minimal storage requirements 
	* 2 bytes per tile for passability requirements.
