<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!-- <a href="https://github.com/Mnzs1701/naomi">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

<h3 align="center">NAOMI (Navigating Autonomously with Offline OSM Maps)</h3>

  <p align="center">
    Naomi is a project that helps autonomous vehicles to drive in regions without good GPS localization. It relies on open street maps in order to establish correspondences with the real world. 
    <br />
    <a href="https://github.com/Mnzs1701/naomi"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <!-- <a href="https://github.com/Mnzs1701/naomi">View Demo</a> -->
    ·
    <a href="https://github.com/Mnzs1701/naomi/issues">Report Bug</a>
    ·
    <a href="https://github.com/Mnzs1701/naomi/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
<!--       <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul> -->
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

Naomi takes in raw camera images along with LiDAR inputs and converts it into a pipeline of lanes, these lanes are used by a particle filter on the OSM Map in order to navigate autonomously in urban environments.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

The following systems need to be set up and running:

1. [LIOSAM](https://github.com/TixiaoShan/LIO-SAM.git) 
2. [lane-detector-ros](https://github.com/Mnzs1701/lane-detector-ros.git)
3. [quick mcl](https://github.com/VorpalBlade/quickmcl.git)
4. docker

### Installation

1. Clone the package and build it 
2. Run the docker for pulling and caching the images, use the yaml file located under the config folder
```
docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Robot Setup
![copernicus](https://github.com/Mnzs1701/naomi/assets/43721824/6e9df660-ac20-4217-83cf-315dd740c966)

Watch some of the demos of the vehicle [here](https://youtu.be/S9IHIZG8YQU?si=RARpwMQrKIiVV4qB)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

System architecture
![Naomi drawio](https://github.com/Mnzs1701/naomi/assets/43721824/9008aae3-b629-47b8-92d1-3956f5fd5a44)


Original location of the testing

![pf_loc](https://github.com/Mnzs1701/naomi/assets/43721824/426cd2a3-2fd1-40e2-a3ab-e7dc3ed06340)

Without the particle Filter

https://github.com/Mnzs1701/naomi/assets/43721824/19c51b0c-0cba-4fc1-9fa8-7e108891eeac

With the particle filter implemented

https://github.com/Mnzs1701/naomi/assets/43721824/8ba2a7f5-ddb0-4175-a6d6-bce4b987d9fb

Further documentation is provided [here](https://drive.google.com/file/d/1w5f-xmZalkNGCKpyGceK2NOm6CGAeIQd/view?usp=sharing)

<!-- ROADMAP -->
<!-- ## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/Mnzs1701/naomi/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the GNU General Public License v3.0 See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Naman Menezes - [NamanMenezes17](https://twitter.com/NamanMenezes17) - email@email_client.com

Project Link: [https://github.com/Mnzs1701/naomi](https://github.com/Mnzs1701/naomi)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* AIRL Lab IISc

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Mnzs1701/naomi.svg?style=for-the-badge
[contributors-url]: https://github.com/Mnzs1701/naomi/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Mnzs1701/naomi.svg?style=for-the-badge
[forks-url]: https://github.com/Mnzs1701/naomi/network/members
[stars-shield]: https://img.shields.io/github/stars/Mnzs1701/naomi.svg?style=for-the-badge
[stars-url]: https://github.com/Mnzs1701/naomi/stargazers
[issues-shield]: https://img.shields.io/github/issues/Mnzs1701/naomi.svg?style=for-the-badge
[issues-url]: https://github.com/Mnzs1701/naomi/issues
[license-shield]: https://img.shields.io/github/license/Mnzs1701/naomi.svg?style=for-the-badge
[license-url]: https://github.com/Mnzs1701/naomi/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/naman-menezes
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
