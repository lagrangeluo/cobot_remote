from typing import List, Generator, Dict, Any

from nerf_vuer.mixins import collector, process_aabb, process_world, collect_rays, chunk_rays
from nerf_vuer.render_nodes import Chainer, RGBA
from vuer.events import ServerEvent
from vuer.schemas import SceneElement

from nerf_vuer.default_settings import RENDER_DEFAULT, RGB_DEFAULT


class Render(SceneElement):
    _fields: Dict[Any, "RenderLayer"]

    tag = "Render"
    key = ""
    # these are the extra options
    # need to be able to register handlers.
    children: List["RenderLayer"] = []

    def __init__(
        self,
        *children,
        layers=[],
        options=[],
        # settings: Controls = Controls(**RENDER_DEFAULT)
        settings=RENDER_DEFAULT,
        **kwargs,
    ):
        super().__init__(
            *children,
            layers=layers,
            options=options,
            settings=settings,
            **kwargs,
        )

    def __post_init__(self):
        self._fields = {}
        for child in self.children:
            # only include those that are RenderLayer
            if not isinstance(child, RenderLayer):
                continue

            self._fields[child.key] = child

    async def render(self, *, camera, world, render, **rest):
        layers = render.get("layers", [])
        # outputs = {}
        for key in layers:
            print(f"layer [{key}]")
            channel_args = render.get(key, {})

            async for render_event in self._fields[key].render(
                camera=camera,
                world=world,
                render=render,
                settings=channel_args,
                **rest,
            ):
                yield render_event


class RenderLayer(SceneElement):
    tag = "RenderLayer"

    channel = "rgb"
    alphaChannel = "alpha"
    displacementMap = None
    distance = 10.1

    settings = RGB_DEFAULT

    def __init__(
        self,
        channel=None,
        alphaChannel=None,
        displacementMap=None,
        geometry="plane",
        settings=None,
        distance=None,
        **kwargs,
    ):
        super().__init__(
            channel=channel or self.channel,
            alphaChannel=alphaChannel or self.alphaChannel,
            displacementMap=displacementMap or self.displacementMap,
            geometry=geometry,
            settings=settings or self.settings,
            distance=distance or self.distance,
            **kwargs,
        )

    def __post_init__(self, _render=None, **_):
        self._render = _render

    @collector(
        pipe=Chainer(RGBA.rgb, RGBA.alpha),
        channels=["rgb", "alpha"],
    )
    @process_aabb
    @process_world
    @collect_rays
    @chunk_rays
    def render(self, ray_bundle, **kwargs) -> Generator[ServerEvent, None, None]:
        return self._render(ray_bundle=ray_bundle, **kwargs)


class Heatmap(RenderLayer):
    tag = "RenderLayer"
    channel = "clip@pca"
    alphaChannel = "clip@mask"


# supports VR and XR 2.5D overlay
# class VRRenderLayer(RenderLayer):
#     tag = "RenderLayer"
#     channel = "clip@pca"
#     alphaChannel = "clip@mask"
#     geometry = "sphere"
