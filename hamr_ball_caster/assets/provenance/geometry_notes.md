# Geometry recovery and confidence

All conversion ran locally. `tools/extract_cad.py` enumerates the native
`Ball Caster Assembly.SLDASM` component tree, reads the matching supplied PART
files, and recovers their saved DisplayLists triangle strips with sldkit 0.2.0.
It exports 16 unique meshes in meters. `extraction.json` records source and mesh
SHA-256 hashes, library versions, analytic CAD surfaces, mesh checks, and parser
limitations. All 16 meshes are closed, consistently wound, and have positive
volume. Each source PART has exactly one saved configuration, whose name and ID
match the configuration selected in the assembly, including vendor hardware.

This is recovery of saved tessellation, not a SolidWorks feature-tree rebuild or
a solid-kernel export. The parser reports partial results; its losses are retained
per part. A closed mesh validates triangle topology, not every recovered CAD
semantic or feature. Unit-density volume, center of mass, and inertia are provided
only for closed meshes; multiply volume and inertia by an independently justified
material density. CAD density, real manufactured mass, surface friction, bearing
drag, and compliance are not established by these geometry files.

## Conflicting saved representations

Several PART meshes differ from the assembly's cached bounding boxes. The main
shaft is now 230 mm long while the assembly cache describes 210 mm. The fork has
up to 9.14 mm of bounding-box difference; the roller and roller shaft differ by
up to 1.61 mm and 1.05 mm. Smaller differences also include tessellation and
bounding-box conventions. Full numeric differences are in `extraction.json`.
The simulation uses the supplied current PART meshes and native occurrence
transforms; it does not silently stretch them to the stale cached bounds.

The saved thumbnail for `Semi-Spherical Wheel.SLDPRT` shows tread-like markings.
Its saved DisplayLists and decoded sphere surface give a smooth spherical band.
There is one saved configuration (`Default`) and its ID matches the assembly.
Whether the thumbnail is stale or depicts an appearance effect cannot be
established here. No tread pattern was invented. Authoritative regenerated
SolidWorks geometry would be needed to resolve that discrepancy.

## Geometry observations

Both shell parts have a 100 mm spherical outer surface centered at their local
origin with symmetry axis +Y. The first PART is the 10–40 mm band along local Y;
the second is the 40–96.8246 mm cap. Two instances of each form the opposing
hemispheres; they are not two copies of a single complete half-sphere.

With current roller meshes at saved assembly positions, rolling-component
vertices reach 101.799 mm from the common sphere center. A nominal 100 mm sphere
is therefore not an exact contact proxy through every carrier angle. In the
saved standalone assembly pose the shell's lowest vertex is -99.954 mm in the
Z-up frame; the pole rollers are above that contact level.

`docs/cad_assembly_views.png` is rendered from the recovered meshes and exact
saved assembly transforms under the documented CAD-to-Z-up rotation. Hardware
with more than 12,000 faces is reduced to 1,200 faces only for this static image.
The meshes used by URDF generation are unchanged. The exploded image translates
the two shell groups by ±95 mm without rotating them; the interior view hides the
four shell parts.
